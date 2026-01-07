#include "global.h"
#include "arm_math.h"
#include "arm_const_structs.h"

// ============================================================================
// CONFIGURATION
// ============================================================================

#define USE_SAMPLING_RATE hz96000
#define FFT_SIZE 256              // 2.67ms @ 96kHz
#define FFT_OVERLAP 128           // 50% Overlap

// Feature-Gewichte (aus Python-Config übernommen)
#define WEIGHT_CENTROID  2.0f
#define WEIGHT_RMS       2.0f
#define WEIGHT_ROLLOFF   1.5f
#define WEIGHT_ZCR       1.5f
#define WEIGHT_FLUX      20.0f
#define WEIGHT_BANDWIDTH 1.0f

// Schwellwerte
#define MIN_AMPLITUDE_THRESHOLD 0.01f     // Minimale RMS für gültige Frames
#define CHANGE_THRESHOLD_PERCENTILE 85.0f // Top 15% als Änderungen
#define MIN_SEGMENT_MS 9.8f               // Minimale Segmentlänge
#define SILENCE_FRAMES_THRESHOLD 0.7f     // Max 70% stille Frames pro Segment

// Clustering
#define MAX_CLUSTERS 4
#define MAX_SEGMENTS 200

// Anti-Click
#define FADE_SAMPLES 288  // 3ms @ 96kHz

// ============================================================================
// DATA STRUCTURES
// ============================================================================

typedef struct {
    float32_t centroid;
    float32_t rms;
    float32_t rolloff;
    float32_t zcr;
    float32_t flux;
    float32_t bandwidth;
    uint8_t valid;  // 1 = gültig, 0 = zu leise (unter Threshold)
} FrameFeatures;

typedef struct {
    float32_t start_time;
    float32_t end_time;
    uint32_t start_sample;
    uint32_t end_sample;
    uint8_t cluster_id;
    float32_t avg_centroid;
    float32_t avg_rms;
} AudioSegment;

typedef struct {
    FrameFeatures features[FFT_SIZE * 10];  // Ringpuffer für Feature-History
    uint32_t write_index;
    uint32_t count;
    
    float32_t prev_spectrum[FFT_SIZE/2];  // Für Spectral Flux
    
    AudioSegment segments[MAX_SEGMENTS];
    uint32_t segment_count;
    
    uint32_t current_segment_start;
    uint8_t in_segment;
    float32_t last_change_time;
} DeinterleavingState;

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

CircularBuffer rx_buffer;
CircularBuffer tx_buffer;

uint32_t in[BLOCK_SIZE];
uint32_t out[BLOCK_SIZE];
int16_t left_in[BLOCK_SIZE];
int16_t right_in[BLOCK_SIZE];
int16_t left_out[BLOCK_SIZE];
int16_t right_out[BLOCK_SIZE];

// FFT Buffers
float32_t fft_input[FFT_SIZE];
float32_t fft_output[FFT_SIZE * 2];
float32_t spectrum_magnitude[FFT_SIZE/2];

// Audio Buffer für FFT-Fenster
float32_t audio_buffer[FFT_SIZE + BLOCK_SIZE];
uint32_t buffer_index = 0;

// Deinterleaving State
DeinterleavingState deint_state;

// FFT Instance
arm_rfft_fast_instance_f32 fft_instance;

// Pre-computed Hanning Window
float32_t hanning_window[FFT_SIZE];

// Output Buffers für separierte Signale (optional für Echtzeit-Playback)
float32_t signal_A[48000 * 60];  // 60 Sekunden @ 48kHz
float32_t signal_B[48000 * 60];
float32_t signal_C[48000 * 60];
uint32_t signal_lengths[3] = {0, 0, 0};

// Performance Monitoring
volatile uint32_t processing_overruns = 0;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

void generate_hanning_window(void) {
    for(uint32_t i = 0; i < FFT_SIZE; i++) {
        hanning_window[i] = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / (FFT_SIZE - 1)));
    }
}

// Fast Normalisierung (Lookup-Table für kleine Werte)
static inline float32_t fast_normalize(int16_t x) {
    return (float32_t)x * 0.000030518f;  // 1/32768
}

// Percentile-Berechnung (vereinfacht für Echtzeit)
float32_t compute_percentile_fast(float32_t* data, uint32_t len, float32_t percentile) {
    if(len == 0) return 0.0f;
    
    // Quick-Select Approximation: Nutze nur jedes 4. Element für Speed
    uint32_t step = (len > 100) ? 4 : 1;
    uint32_t sample_count = len / step;
    
    float32_t samples[256];  // Max 256 Samples für Percentile
    if(sample_count > 256) sample_count = 256;
    
    for(uint32_t i = 0; i < sample_count; i++) {
        samples[i] = data[i * step];
    }
    
    // Simple Bubble-Sort für kleine Arrays (schnell genug)
    for(uint32_t i = 0; i < sample_count - 1; i++) {
        for(uint32_t j = 0; j < sample_count - i - 1; j++) {
            if(samples[j] > samples[j+1]) {
                float32_t temp = samples[j];
                samples[j] = samples[j+1];
                samples[j+1] = temp;
            }
        }
    }
    
    uint32_t idx = (uint32_t)((percentile / 100.0f) * sample_count);
    if(idx >= sample_count) idx = sample_count - 1;
    
    return samples[idx];
}

// ============================================================================
// FEATURE EXTRACTION
// ============================================================================

void compute_frame_features(float32_t* audio_frame, FrameFeatures* features, 
                            float32_t* prev_spec, float32_t sample_rate) {
    
    // 1. RMS
    float32_t rms_sq;
    arm_rms_f32(audio_frame, FFT_SIZE, &rms_sq);
    features->rms = rms_sq;
    
    // Prüfe Amplitudenschwelle (wie in Python: MIN_AMPLITUDE_THRESHOLD)
    if(features->rms < MIN_AMPLITUDE_THRESHOLD) {
        features->valid = 0;
        // Setze alle Features auf 0 für ungültige Frames
        features->centroid = 0.0f;
        features->rolloff = 0.0f;
        features->zcr = 0.0f;
        features->flux = 0.0f;
        features->bandwidth = 0.0f;
        return;
    }
    
    features->valid = 1;
    
    // 2. Zero Crossing Rate
    uint32_t zero_crossings = 0;
    for(uint32_t i = 1; i < FFT_SIZE; i++) {
        if((audio_frame[i-1] >= 0.0f && audio_frame[i] < 0.0f) ||
           (audio_frame[i-1] < 0.0f && audio_frame[i] >= 0.0f)) {
            zero_crossings++;
        }
    }
    features->zcr = (float32_t)zero_crossings / (float32_t)FFT_SIZE;
    
    // 3. Window anwenden
    arm_mult_f32(audio_frame, hanning_window, fft_input, FFT_SIZE);
    
    // 4. FFT
    arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0);
    
    // 5. Magnitude Spectrum
    arm_cmplx_mag_f32(fft_output, spectrum_magnitude, FFT_SIZE/2);
    
    // 6. Spectral Centroid
    float32_t weighted_sum = 0.0f;
    float32_t total_power = 0.0f;
    float32_t freq_resolution = sample_rate / (float32_t)FFT_SIZE;
    
    for(uint32_t i = 0; i < FFT_SIZE/2; i++) {
        float32_t power = spectrum_magnitude[i] * spectrum_magnitude[i];
        float32_t freq = (float32_t)i * freq_resolution;
        weighted_sum += freq * power;
        total_power += power;
    }
    
    features->centroid = (total_power > 1e-10f) ? (weighted_sum / total_power) : 0.0f;
    
    // 7. Spectral Rolloff (85% der Energie)
    if(total_power > 1e-10f) {
        float32_t cumsum = 0.0f;
        float32_t threshold_power = 0.85f * total_power;
        
        for(uint32_t i = 0; i < FFT_SIZE/2; i++) {
            cumsum += spectrum_magnitude[i] * spectrum_magnitude[i];
            if(cumsum >= threshold_power) {
                features->rolloff = (float32_t)i * freq_resolution;
                break;
            }
        }
    } else {
        features->rolloff = 0.0f;
    }
    
    // 8. Spectral Bandwidth
    if(total_power > 1e-10f) {
        float32_t variance = 0.0f;
        for(uint32_t i = 0; i < FFT_SIZE/2; i++) {
            float32_t power = spectrum_magnitude[i] * spectrum_magnitude[i];
            float32_t freq = (float32_t)i * freq_resolution;
            float32_t diff = freq - features->centroid;
            variance += diff * diff * power;
        }
        arm_sqrt_f32(variance / total_power, &features->bandwidth);
    } else {
        features->bandwidth = 0.0f;
    }
    
    // 9. Spectral Flux (Änderung zum vorherigen Frame)
    if(prev_spec != NULL) {
        float32_t flux_sum = 0.0f;
        for(uint32_t i = 0; i < FFT_SIZE/2; i++) {
            float32_t diff = spectrum_magnitude[i] - prev_spec[i];
            flux_sum += diff * diff;
        }
        features->flux = flux_sum;
    } else {
        features->flux = 0.0f;
    }
    
    // Aktualisiere prev_spectrum für nächsten Frame
    if(prev_spec != NULL) {
        arm_copy_f32(spectrum_magnitude, prev_spec, FFT_SIZE/2);
    }
}

// ============================================================================
// CHANGE DETECTION (wie in Python: detect_changes)
// ============================================================================

float32_t compute_change_score(FrameFeatures* current, FrameFeatures* previous) {
    
    // Wenn einer der Frames ungültig ist, Return 0
    if(!current->valid || !previous->valid) {
        return 0.0f;
    }
    
    // Normalisierung (vereinfacht - nutze globale Min/Max aus History)
    // In Production: Diese Werte aus einem Kalibrierungs-Fenster berechnen
    
    const float32_t CENT_SCALE = 1.0f / 10000.0f;  // Typ. Range: 0-10kHz
    const float32_t RMS_SCALE = 10.0f;             // Typ. Range: 0-0.1
    const float32_t ROLL_SCALE = 1.0f / 20000.0f;  // Typ. Range: 0-20kHz
    const float32_t ZCR_SCALE = 5.0f;              // Typ. Range: 0-0.2
    const float32_t FLUX_SCALE = 0.01f;            // Typ. Range: 0-100
    const float32_t BW_SCALE = 1.0f / 5000.0f;     // Typ. Range: 0-5kHz
    
    // Änderungen berechnen (absolute Differenzen)
    float32_t cent_change = fabsf(current->centroid - previous->centroid) * CENT_SCALE;
    float32_t rms_change = fabsf(current->rms - previous->rms) * RMS_SCALE;
    float32_t roll_change = fabsf(current->rolloff - previous->rolloff) * ROLL_SCALE;
    float32_t zcr_change = fabsf(current->zcr - previous->zcr) * ZCR_SCALE;
    float32_t flux_change = fabsf(current->flux - previous->flux) * FLUX_SCALE;
    float32_t bw_change = fabsf(current->bandwidth - previous->bandwidth) * BW_SCALE;
    
    // Clip auf [0, 1]
    cent_change = (cent_change > 1.0f) ? 1.0f : cent_change;
    rms_change = (rms_change > 1.0f) ? 1.0f : rms_change;
    roll_change = (roll_change > 1.0f) ? 1.0f : roll_change;
    zcr_change = (zcr_change > 1.0f) ? 1.0f : zcr_change;
    flux_change = (flux_change > 1.0f) ? 1.0f : flux_change;
    bw_change = (bw_change > 1.0f) ? 1.0f : bw_change;
    
    // Gewichtete Summe (wie in Python)
    float32_t total_weight = WEIGHT_CENTROID + WEIGHT_RMS + WEIGHT_ROLLOFF + 
                            WEIGHT_ZCR + WEIGHT_FLUX + WEIGHT_BANDWIDTH;
    
    float32_t combined = (
        WEIGHT_CENTROID * cent_change +
        WEIGHT_RMS * rms_change +
        WEIGHT_ROLLOFF * roll_change +
        WEIGHT_ZCR * zcr_change +
        WEIGHT_FLUX * flux_change +
        WEIGHT_BANDWIDTH * bw_change
    ) / total_weight;
    
    return combined;
}

// ============================================================================
// SEGMENT DETECTION & CLUSTERING
// ============================================================================

void detect_segment_boundary(DeinterleavingState* state, float32_t change_score, 
                            float32_t current_time, uint32_t current_sample) {
    
    // Dynamischer Threshold aus Feature-History (wie Python Percentile)
    static float32_t change_history[100];
    static uint32_t history_index = 0;
    static uint32_t history_count = 0;
    
    change_history[history_index] = change_score;
    history_index = (history_index + 1) % 100;
    if(history_count < 100) history_count++;
    
    float32_t threshold = compute_percentile_fast(change_history, history_count, 
                                                  CHANGE_THRESHOLD_PERCENTILE);
    
    // Segment-Detektion
    if(change_score > threshold && change_score > 0.1f) {
        
        // Prüfe Mindest-Segmentlänge (MIN_SEGMENT_MS)
        float32_t segment_duration = current_time - state->last_change_time;
        
        if(segment_duration * 1000.0f >= MIN_SEGMENT_MS) {
            
            // Neues Segment starten
            if(state->in_segment && state->segment_count < MAX_SEGMENTS) {
                
                // Altes Segment abschließen
                AudioSegment* seg = &state->segments[state->segment_count];
                seg->start_sample = state->current_segment_start;
                seg->end_sample = current_sample;
                seg->start_time = state->last_change_time;
                seg->end_time = current_time;
                
                // Durchschnitts-Features berechnen (vereinfacht)
                // In vollständiger Implementation: Über alle Frames im Segment mitteln
                seg->avg_centroid = 0.0f;  // TODO: Aus Feature-History berechnen
                seg->avg_rms = 0.0f;
                seg->cluster_id = 0;  // Wird später beim Clustering gesetzt
                
                state->segment_count++;
            }
            
            // Neues Segment beginnen
            state->current_segment_start = current_sample;
            state->in_segment = 1;
            state->last_change_time = current_time;
        }
    }
}

// Vereinfachtes Clustering (K-Means-ähnlich für 2-3 Cluster)
void cluster_segments_simple(DeinterleavingState* state) {
    
    if(state->segment_count < 2) {
        // Nur 1 Segment: Alle zu Cluster 0
        for(uint32_t i = 0; i < state->segment_count; i++) {
            state->segments[i].cluster_id = 0;
        }
        return;
    }
    
    // Berechne Centroid-Range (wie in Python)
    float32_t min_cent = 99999.0f;
    float32_t max_cent = 0.0f;
    
    for(uint32_t i = 0; i < state->segment_count; i++) {
        float32_t cent = state->segments[i].avg_centroid;
        if(cent < min_cent) min_cent = cent;
        if(cent > max_cent) max_cent = cent;
    }
    
    float32_t cent_range = max_cent - min_cent;
    
    // Entscheide Cluster-Anzahl (wie Python-Logik)
    uint32_t n_clusters = 1;
    if(cent_range > 3000.0f) {
        n_clusters = 3;
    } else if(cent_range > 1000.0f) {
        n_clusters = 2;
    }
    
    // Einfaches Clustering: Sortiere nach Centroid und teile in Bereiche
    if(n_clusters == 1) {
        for(uint32_t i = 0; i < state->segment_count; i++) {
            state->segments[i].cluster_id = 0;
        }
    } else {
        float32_t range_per_cluster = cent_range / (float32_t)n_clusters;
        
        for(uint32_t i = 0; i < state->segment_count; i++) {
            float32_t cent = state->segments[i].avg_centroid;
            uint32_t cluster = (uint32_t)((cent - min_cent) / range_per_cluster);
            if(cluster >= n_clusters) cluster = n_clusters - 1;
            state->segments[i].cluster_id = (uint8_t)cluster;
        }
    }
}

// ============================================================================
// ANTI-CLICK: ZERO-CROSSING ALIGNMENT + FADE
// ============================================================================

int32_t find_zero_crossing(int16_t* audio, int32_t idx, int32_t search_range) {
    
    int32_t start = (idx - search_range < 0) ? 0 : (idx - search_range);
    int32_t end = idx + search_range;
    
    int32_t best_idx = idx;
    int16_t min_abs = 32767;
    
    for(int32_t i = start; i < end - 1; i++) {
        // Suche Vorzeichenwechsel mit minimalem Wert
        if((audio[i] >= 0 && audio[i+1] < 0) || (audio[i] < 0 && audio[i+1] >= 0)) {
            int16_t abs_val = (audio[i] > 0) ? audio[i] : -audio[i];
            if(abs_val < min_abs) {
                min_abs = abs_val;
                best_idx = i;
            }
        }
    }
    
    return best_idx;
}

void apply_fade(int16_t* buffer, uint32_t length, uint8_t fade_in) {
    
    uint32_t fade_len = (length < FADE_SAMPLES) ? (length / 4) : FADE_SAMPLES;
    
    for(uint32_t i = 0; i < fade_len; i++) {
        float32_t factor;
        
        if(fade_in) {
            // Fade In: Cosine Window
            factor = (1.0f - arm_cos_f32(PI * (float32_t)i / (float32_t)fade_len)) / 2.0f;
        } else {
            // Fade Out
            factor = (1.0f + arm_cos_f32(PI * (float32_t)i / (float32_t)fade_len)) / 2.0f;
        }
        
        if(fade_in) {
            buffer[i] = (int16_t)((float32_t)buffer[i] * factor);
        } else {
            buffer[length - 1 - i] = (int16_t)((float32_t)buffer[length - 1 - i] * factor);
        }
    }
}

// ============================================================================
// MAIN
// ============================================================================

int main(){
    init_platform(115200, USE_SAMPLING_RATE, line_in);
    
    debug_printf("%s, %s\n", __DATE__, __TIME__);
    IF_DEBUG(debug_printf("=== REAL-TIME AUDIO DEINTERLEAVING ===\n"));
    IF_DEBUG(debug_printf("Sample Rate: 96kHz\n"));
    IF_DEBUG(debug_printf("FFT Size: %d (%.2f ms)\n", FFT_SIZE, 
             (float)FFT_SIZE * 1000.0f / 96000.0f));
    IF_DEBUG(debug_printf("Min Segment: %.1f ms\n", MIN_SEGMENT_MS));
    
    gpio_set(TEST_PIN, LOW);
    
    rx_buffer.init();
    tx_buffer.init();
    memset(in, 0, sizeof(in));
    memset(out, 0, sizeof(out));
    
    // FFT Init
    arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);
    generate_hanning_window();
    
    // State Init
    memset(&deint_state, 0, sizeof(DeinterleavingState));
    memset(audio_buffer, 0, sizeof(audio_buffer));
    buffer_index = 0;
    
    platform_start();
    
    IF_DEBUG(debug_printf("Processing started...\n"));
    
    uint32_t frame_counter = 0;
    float32_t current_time = 0.0f;
    
    while(true)
    {
        if(!rx_buffer.read(in)) {
            continue;
        }
        
        gpio_set(LED_B, HIGH);
        gpio_set(TEST_PIN, HIGH);
        
        convert_audio_sample_to_2ch(in, left_in, right_in);
        
        // ========== FEATURE EXTRACTION & DEINTERLEAVING ==========
        
        // Akkumuliere Samples für FFT-Fenster
        for(uint32_t n = 0; n < BLOCK_SIZE; n++) {
            if(buffer_index < FFT_SIZE) {
                // Nutze linken Kanal für Mono-Analyse
                audio_buffer[buffer_index] = fast_normalize(left_in[n]);
                buffer_index++;
            }
        }
        
        // Wenn FFT-Fenster voll: Feature-Extraktion
        if(buffer_index >= FFT_SIZE) {
            
            FrameFeatures current_features;
            
            // Berechne Features für aktuellen Frame
            compute_frame_features(audio_buffer, &current_features, 
                                 deint_state.prev_spectrum, 96000.0f);
            
            // Speichere Features in History
            uint32_t hist_idx = deint_state.write_index;
            deint_state.features[hist_idx] = current_features;
            deint_state.write_index = (deint_state.write_index + 1) % (FFT_SIZE * 10);
            if(deint_state.count < FFT_SIZE * 10) deint_state.count++;
            
            // Change Detection (wenn vorheriger Frame existiert)
            if(frame_counter > 0) {
                uint32_t prev_idx = (hist_idx > 0) ? (hist_idx - 1) : (FFT_SIZE * 10 - 1);
                FrameFeatures* prev_features = &deint_state.features[prev_idx];
                
                float32_t change_score = compute_change_score(&current_features, prev_features);
                
                // Segment Boundary Detection
                detect_segment_boundary(&deint_state, change_score, current_time,
                                      frame_counter * FFT_OVERLAP);
            }
            
            frame_counter++;
            current_time = (float32_t)(frame_counter * FFT_OVERLAP) / 96000.0f;
            
            // Overlap: Buffer verschieben
            memmove(audio_buffer, &audio_buffer[FFT_OVERLAP], 
                   (FFT_SIZE - FFT_OVERLAP) * sizeof(float32_t));
            buffer_index = FFT_SIZE - FFT_OVERLAP;
        }
        
        // ========== OUTPUT: PASSTHROUGH (oder Segment-basierter Routing) ==========
        
        // Echtzeit-Routing zu separaten Ausgängen
        // Wenn Segmente erkannt wurden, route zu entsprechendem Cluster
        if(deint_state.segment_count > 0) {
            // Finde aktuelles Segment
            uint32_t current_sample = frame_counter * BLOCK_SIZE;
            uint8_t current_cluster = 0;
            
            for(uint32_t i = 0; i < deint_state.segment_count; i++) {
                AudioSegment* seg = &deint_state.segments[i];
                if(current_sample >= seg->start_sample && current_sample < seg->end_sample) {
                    current_cluster = seg->cluster_id;
                    break;
                }
            }
            
          // ========== OUTPUT: Segment-basierter Routing zu Signal A ==========

            // Standard: Passthrough während keine Segmente erkannt wurden
            if(deint_state.segment_count == 0) {
                // Initial: Passthrough bis erste Segmente erkannt sind
                for(uint32_t n = 0; n < BLOCK_SIZE; n++) {
                    left_out[n] = left_in[n];
                    right_out[n] = right_in[n];
                }
            } else {
                // Finde aktuelles Segment basierend auf Sample-Position
                uint32_t current_sample = frame_counter * FFT_OVERLAP;  // Nicht BLOCK_SIZE!
                uint8_t current_cluster = 0;
                uint8_t found_segment = 0;
                
                for(uint32_t i = 0; i < deint_state.segment_count; i++) {
                    AudioSegment* seg = &deint_state.segments[i];
                    if(current_sample >= seg->start_sample && current_sample < seg->end_sample) {
                        current_cluster = seg->cluster_id;
                        found_segment = 1;
                        break;
                    }
                }
                
                // Route basierend auf Cluster
                if(found_segment && current_cluster == 0) {
                    // Signal A: Ausgeben
                    for(uint32_t n = 0; n < BLOCK_SIZE; n++) {
                        left_out[n] = left_in[n];
                        right_out[n] = right_in[n];
                    }
                } else {
                    // Signal B/C oder unbekannt: Stummschalten
                    memset(left_out, 0, BLOCK_SIZE * sizeof(int16_t));
                    memset(right_out, 0, BLOCK_SIZE * sizeof(int16_t));
                }
            }

            // Audio-Samples zu Stereo-Format konvertieren
            convert_2ch_to_audio_sample(left_out, right_out, out);
        
        
        // convert_2ch_to_audio_sample(left_out, right_out, out);
        
        if(!tx_buffer.write(out)) {
            processing_overruns++;
        }
        
        gpio_set(LED_B, LOW);
        gpio_set(TEST_PIN, LOW);
        
        // ========== PERIODIC STATUS OUTPUT ==========
        #ifdef DEBUG
        static uint32_t status_counter = 0;
        if(++status_counter >= 96000 / BLOCK_SIZE) {  // Jede Sekunde
            debug_printf("Segments: %lu, Time: %.1fs\n", 
                        deint_state.segment_count, current_time);
            status_counter = 0;
        }
        #endif
    }
    
    fatal_error();
    return 0;
   }
}
uint32_t* get_new_tx_buffer_ptr(){
    uint32_t* temp = tx_buffer.get_read_ptr();
    if(temp == NULL) fatal_error();
    return temp;
}

uint32_t* get_new_rx_buffer_ptr(){
    uint32_t* temp = rx_buffer.get_write_ptr();
    if(temp == NULL) fatal_error();
    return temp;
}