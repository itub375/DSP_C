#include "global.h"
#include "arm_math.h"
#include "arm_const_structs.h"

// ============================================================================
// CONFIGURATION
// ============================================================================

#define USE_SAMPLING_RATE hz96000
#define FFT_SIZE 256              // 2.67ms @ 96kHz
#define FFT_OVERLAP 128           // 50% Overlap

// Feature-Gewichte
#define WEIGHT_CENTROID  2.0f
#define WEIGHT_RMS       2.0f
#define WEIGHT_ROLLOFF   1.5f
#define WEIGHT_ZCR       1.5f
#define WEIGHT_FLUX      20.0f
#define WEIGHT_BANDWIDTH 1.0f

// Schwellwerte
#define MIN_AMPLITUDE_THRESHOLD 0.01f
#define CHANGE_THRESHOLD_PERCENTILE 85.0f
#define MIN_SEGMENT_MS 9.8f

// Clustering
#define MAX_CLUSTERS 4
#define MAX_SEGMENTS 400

// Anti-Click
#define FADE_SAMPLES 288  // 3ms @ 96kHz

// Recording Buffer (60 Sekunden @ 96kHz Mono)
#define MAX_RECORDING_SAMPLES (96000 * 60)

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
    uint8_t valid;
} FrameFeatures;

typedef struct {
    float32_t start_time;
    float32_t end_time;
    uint32_t start_sample;
    uint32_t end_sample;
    uint8_t cluster_id;
    float32_t avg_centroid;
    float32_t avg_rms;
    float32_t avg_rolloff;
    uint32_t feature_start_idx;  // Index im Feature-Array
    uint32_t feature_count;      // Anzahl Features in diesem Segment
} AudioSegment;

typedef struct {
    FrameFeatures features[FFT_SIZE * 10];
    uint32_t write_index;
    uint32_t count;
    
    float32_t prev_spectrum[FFT_SIZE/2];
    
    AudioSegment segments[MAX_SEGMENTS];
    uint32_t segment_count;
    
    uint32_t current_segment_start;
    uint8_t in_segment;
    float32_t last_change_time;
} DeinterleavingState;

typedef enum {
    MODE_RECORDING,    // Pass 1: Nimmt Audio auf und analysiert
    MODE_PROCESSING,   // Clustering & Segment-Finalisierung
    MODE_PLAYBACK      // Pass 2: Spielt separierte Signale ab
} ProcessingMode;

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

// Recording Buffer (für gesamtes Audio)
int16_t recorded_audio[MAX_RECORDING_SAMPLES];
uint32_t recorded_samples = 0;

// Deinterleaving State
DeinterleavingState deint_state;

// Processing Mode
ProcessingMode current_mode = MODE_RECORDING;

// Playback State
uint32_t playback_position = 0;
uint8_t playback_cluster = 0;  // Welches Cluster wird abgespielt (0, 1, 2)

// FFT Instance
arm_rfft_fast_instance_f32 fft_instance;

// Pre-computed Hanning Window
float32_t hanning_window[FFT_SIZE];

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

static inline float32_t fast_normalize(int16_t x) {
    return (float32_t)x * 0.000030518f;  // 1/32768
}

float32_t compute_percentile_fast(float32_t* data, uint32_t len, float32_t percentile) {
    if(len == 0) return 0.0f;
    
    uint32_t step = (len > 100) ? 4 : 1;
    uint32_t sample_count = len / step;
    
    float32_t samples[256];
    if(sample_count > 256) sample_count = 256;
    
    for(uint32_t i = 0; i < sample_count; i++) {
        samples[i] = data[i * step];
    }
    
    // Bubble-Sort
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
    
    if(features->rms < MIN_AMPLITUDE_THRESHOLD) {
        features->valid = 0;
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
    
    // 7. Spectral Rolloff
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
    
    // 9. Spectral Flux
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
    
    if(prev_spec != NULL) {
        arm_copy_f32(spectrum_magnitude, prev_spec, FFT_SIZE/2);
    }
}

// ============================================================================
// CHANGE DETECTION
// ============================================================================

float32_t compute_change_score(FrameFeatures* current, FrameFeatures* previous) {
    
    if(!current->valid || !previous->valid) {
        return 0.0f;
    }
    
    const float32_t CENT_SCALE = 1.0f / 10000.0f;
    const float32_t RMS_SCALE = 10.0f;
    const float32_t ROLL_SCALE = 1.0f / 20000.0f;
    const float32_t ZCR_SCALE = 5.0f;
    const float32_t FLUX_SCALE = 0.01f;
    const float32_t BW_SCALE = 1.0f / 5000.0f;
    
    float32_t cent_change = fabsf(current->centroid - previous->centroid) * CENT_SCALE;
    float32_t rms_change = fabsf(current->rms - previous->rms) * RMS_SCALE;
    float32_t roll_change = fabsf(current->rolloff - previous->rolloff) * ROLL_SCALE;
    float32_t zcr_change = fabsf(current->zcr - previous->zcr) * ZCR_SCALE;
    float32_t flux_change = fabsf(current->flux - previous->flux) * FLUX_SCALE;
    float32_t bw_change = fabsf(current->bandwidth - previous->bandwidth) * BW_SCALE;
    
    cent_change = (cent_change > 1.0f) ? 1.0f : cent_change;
    rms_change = (rms_change > 1.0f) ? 1.0f : rms_change;
    roll_change = (roll_change > 1.0f) ? 1.0f : roll_change;
    zcr_change = (zcr_change > 1.0f) ? 1.0f : zcr_change;
    flux_change = (flux_change > 1.0f) ? 1.0f : flux_change;
    bw_change = (bw_change > 1.0f) ? 1.0f : bw_change;
    
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
// SEGMENT DETECTION
// ============================================================================

void detect_segment_boundary(DeinterleavingState* state, float32_t change_score, 
                            float32_t current_time, uint32_t current_sample,
                            uint32_t feature_idx) {
    
    static float32_t change_history[100];
    static uint32_t history_index = 0;
    static uint32_t history_count = 0;
    
    change_history[history_index] = change_score;
    history_index = (history_index + 1) % 100;
    if(history_count < 100) history_count++;
    
    float32_t threshold = compute_percentile_fast(change_history, history_count, 
                                                  CHANGE_THRESHOLD_PERCENTILE);
    
    if(change_score > threshold && change_score > 0.1f) {
        
        float32_t segment_duration = current_time - state->last_change_time;
        
        if(segment_duration * 1000.0f >= MIN_SEGMENT_MS) {
            
            if(state->in_segment && state->segment_count < MAX_SEGMENTS) {
                
                // Altes Segment abschließen
                AudioSegment* seg = &state->segments[state->segment_count];
                seg->start_sample = state->current_segment_start;
                seg->end_sample = current_sample;
                seg->start_time = state->last_change_time;
                seg->end_time = current_time;
                seg->feature_count = feature_idx - seg->feature_start_idx;
                
                // WICHTIG: Berechne echte Durchschnitts-Features aus History
                float32_t sum_cent = 0.0f;
                float32_t sum_rms = 0.0f;
                float32_t sum_roll = 0.0f;
                uint32_t valid_count = 0;
                
                for(uint32_t i = 0; i < seg->feature_count; i++) {
                    uint32_t idx = (seg->feature_start_idx + i) % (FFT_SIZE * 10);
                    FrameFeatures* feat = &state->features[idx];
                    if(feat->valid) {
                        sum_cent += feat->centroid;
                        sum_rms += feat->rms;
                        sum_roll += feat->rolloff;
                        valid_count++;
                    }
                }
                
                if(valid_count > 0) {
                    seg->avg_centroid = sum_cent / (float32_t)valid_count;
                    seg->avg_rms = sum_rms / (float32_t)valid_count;
                    seg->avg_rolloff = sum_roll / (float32_t)valid_count;
                } else {
                    seg->avg_centroid = 0.0f;
                    seg->avg_rms = 0.0f;
                    seg->avg_rolloff = 0.0f;
                }
                
                seg->cluster_id = 0;
                
                state->segment_count++;
            }
            
            // Neues Segment beginnen
            state->current_segment_start = current_sample;
            state->in_segment = 1;
            state->last_change_time = current_time;
            
            if(state->segment_count < MAX_SEGMENTS) {
                state->segments[state->segment_count].feature_start_idx = feature_idx;
            }
        }
    }
}

// ============================================================================
// CLUSTERING (K-Means-ähnlich)
// ============================================================================

void cluster_segments_simple(DeinterleavingState* state) {
    
    if(state->segment_count < 2) {
        for(uint32_t i = 0; i < state->segment_count; i++) {
            state->segments[i].cluster_id = 0;
        }
        return;
    }
    
    // Finde Min/Max Centroid
    float32_t min_cent = 99999.0f;
    float32_t max_cent = 0.0f;
    
    for(uint32_t i = 0; i < state->segment_count; i++) {
        float32_t cent = state->segments[i].avg_centroid;
        if(cent > 0.0f) {  // Nur gültige Werte
            if(cent < min_cent) min_cent = cent;
            if(cent > max_cent) max_cent = cent;
        }
    }
    
    float32_t cent_range = max_cent - min_cent;
    
    // Entscheide Cluster-Anzahl
    uint32_t n_clusters = 1;
    if(cent_range > 3000.0f) {
        n_clusters = 3;
    } else if(cent_range > 1000.0f) {
        n_clusters = 2;
    }
    
    IF_DEBUG(debug_printf("Clustering: %lu segments, range: %.1f Hz, clusters: %lu\n",
                          state->segment_count, cent_range, n_clusters));
    
    if(n_clusters == 1) {
        for(uint32_t i = 0; i < state->segment_count; i++) {
            state->segments[i].cluster_id = 0;
        }
    } else {
        float32_t range_per_cluster = cent_range / (float32_t)n_clusters;
        
        for(uint32_t i = 0; i < state->segment_count; i++) {
            float32_t cent = state->segments[i].avg_centroid;
            if(cent > 0.0f) {
                uint32_t cluster = (uint32_t)((cent - min_cent) / range_per_cluster);
                if(cluster >= n_clusters) cluster = n_clusters - 1;
                state->segments[i].cluster_id = (uint8_t)cluster;
            } else {
                state->segments[i].cluster_id = 0;
            }
        }
    }
}

// ============================================================================
// ANTI-CLICK FADE
// ============================================================================

void apply_crossfade(int16_t* buffer_out, int16_t* buffer_in, uint32_t fade_len) {
    for(uint32_t i = 0; i < fade_len; i++) {
        float32_t fade_out = (1.0f + arm_cos_f32(PI * (float32_t)i / (float32_t)fade_len)) / 2.0f;
        float32_t fade_in = 1.0f - fade_out;
        
        buffer_out[i] = (int16_t)((float32_t)buffer_out[i] * fade_out + 
                                   (float32_t)buffer_in[i] * fade_in);
    }
}

// ============================================================================
// MAIN
// ============================================================================

int main()
{
    init_platform(115200, USE_SAMPLING_RATE, line_in);
    
    debug_printf("%s, %s\n", __DATE__, __TIME__);
    IF_DEBUG(debug_printf("=== DUAL-PASS AUDIO DEINTERLEAVING ===\n"));
    IF_DEBUG(debug_printf("Sample Rate: 96kHz\n"));
    IF_DEBUG(debug_printf("FFT Size: %d (%.2f ms)\n", FFT_SIZE, 
             (float)FFT_SIZE * 1000.0f / 96000.0f));
    IF_DEBUG(debug_printf("Max Recording: %d seconds\n", MAX_RECORDING_SAMPLES / 96000));
    
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
    memset(recorded_audio, 0, sizeof(recorded_audio));
    buffer_index = 0;
    recorded_samples = 0;
    
    platform_start();
    
    IF_DEBUG(debug_printf("PASS 1: Recording started... (Press button to stop)\n"));
    
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
        
        // ========== MODE-ABHÄNGIGE VERARBEITUNG ==========
        
        if(current_mode == MODE_RECORDING) {
            
            // PASS 1: Aufnahme & Analyse
            
            // Speichere Audio in Buffer
            for(uint32_t n = 0; n < BLOCK_SIZE; n++) {
                if(recorded_samples < MAX_RECORDING_SAMPLES) {
                    recorded_audio[recorded_samples++] = left_in[n];
                }
            }
            
            // Feature-Extraktion
            for(uint32_t n = 0; n < BLOCK_SIZE; n++) {
                if(buffer_index < FFT_SIZE) {
                    audio_buffer[buffer_index] = fast_normalize(left_in[n]);
                    buffer_index++;
                }
            }
            
            if(buffer_index >= FFT_SIZE) {
                
                FrameFeatures current_features;
                
                compute_frame_features(audio_buffer, &current_features, 
                                     deint_state.prev_spectrum, 96000.0f);
                
                uint32_t hist_idx = deint_state.write_index;
                deint_state.features[hist_idx] = current_features;
                deint_state.write_index = (deint_state.write_index + 1) % (FFT_SIZE * 10);
                if(deint_state.count < FFT_SIZE * 10) deint_state.count++;
                
                if(frame_counter > 0) {
                    uint32_t prev_idx = (hist_idx > 0) ? (hist_idx - 1) : (FFT_SIZE * 10 - 1);
                    FrameFeatures* prev_features = &deint_state.features[prev_idx];
                    
                    float32_t change_score = compute_change_score(&current_features, prev_features);
                    
                    detect_segment_boundary(&deint_state, change_score, current_time,
                                          frame_counter * FFT_OVERLAP, hist_idx);
                }
                
                frame_counter++;
                current_time = (float32_t)(frame_counter * FFT_OVERLAP) / 96000.0f;
                
                memmove(audio_buffer, &audio_buffer[FFT_OVERLAP], 
                       (FFT_SIZE - FFT_OVERLAP) * sizeof(float32_t));
                buffer_index = FFT_SIZE - FFT_OVERLAP;
            }
            
            // Passthrough während Aufnahme
            for(uint32_t n = 0; n < BLOCK_SIZE; n++) {
                left_out[n] = left_in[n];
                right_out[n] = right_in[n];
            }
            
            // Check: Aufnahme beenden (z.B. nach 30 Sekunden oder Button-Press)
            if(recorded_samples >= 96000 * 30 || gpio_get(BUTTON_A)) {
                
                IF_DEBUG(debug_printf("\nPASS 1 Complete: %lu samples, %lu segments\n", 
                                    recorded_samples, deint_state.segment_count));
                
                // Schließe letztes Segment ab
                if(deint_state.in_segment && deint_state.segment_count < MAX_SEGMENTS) {
                    AudioSegment* seg = &deint_state.segments[deint_state.segment_count];
                    seg->end_sample = recorded_samples;
                    seg->end_time = (float32_t)recorded_samples / 96000.0f;
                    deint_state.segment_count++;
                }
                
                current_mode = MODE_PROCESSING;
                IF_DEBUG(debug_printf("PASS 2: Processing...\n"));
            }
            
        } else if(current_mode == MODE_PROCESSING) {
            
            // Clustering durchführen
            cluster_segments_simple(&deint_state);
            
            IF_DEBUG(debug_printf("Clustering complete. Starting playback...\n"));
            IF_DEBUG(debug_printf("Cluster 0 segments: "));
            uint32_t cluster_0_count = 0;
            for(uint32_t i = 0; i < deint_state.segment_count; i++) {
                if(deint_state.segments[i].cluster_id == 0) {
                    cluster_0_count++;
                }
            }
            IF_DEBUG(debug_printf("%lu\n", cluster_0_count));
            
            playback_position = 0;
            playback_cluster = 0;  // Spiele Cluster 0 ab (Signal A)
            current_mode = MODE_PLAYBACK;
            
        } else if(current_mode == MODE_PLAYBACK) {
            
            // PASS 2: Playback des separierten Signals
            
            for(uint32_t n = 0; n < BLOCK_SIZE; n++) {
                
                if(playback_position >= recorded_samples) {
                    // Playback beendet - Loop oder Stop
                    playback_position = 0;
                    // Optional: current_mode = MODE_RECORDING; für Neuaufnahme
                }
                
                // Finde aktuelles Segment
                uint8_t should_play = 0;
                
                for(uint32_t i = 0; i < deint_state.segment_count; i++) {
                    AudioSegment* seg = &deint_state.segments[i];
                    if(playback_position >= seg->start_sample && 
                       playback_position < seg->end_sample) {
                        if(seg->cluster_id == playback_cluster) {
                            should_play = 1;
                        }
                        break;
                    }
                }
                
                if(should_play) {
                    left_out[n] = recorded_audio[playback_position];
                    right_out[n] = recorded_audio[playback_position];
                } else {
                    left_out[n] = 0;
                    right_out[n] = 0;
                }
                
                playback_position++;
            }
        }
        
        // Output
        convert_2ch_to_audio_sample(left_out, right_out, out);
        
        if(!tx_buffer.write(out)) {
            processing_overruns++;
        }
        
        gpio_set(LED_B, LOW);
        gpio_set(TEST_PIN, LOW);
        
        // Status Output
        #ifdef DEBUG
        static uint32_t status_counter = 0;
        if(++status_counter >= 96000 / BLOCK_SIZE) {
            if(current_mode == MODE_RECORDING) {
                debug_printf("Recording: %.1fs, Segments: %lu\n", 
                            current_time, deint_state.segment_count);
            } else if(current_mode == MODE_PLAYBACK) {
                debug_printf("Playback: %.1fs / %.1fs\n", 
                            (float)playback_position / 96000.0f,
                            (float)recorded_samples / 96000.0f);
            }
            status_counter = 0;
        }
        #endif
    }
    
    fatal_error();
    return 0;
}

uint32_t* get_new_tx_buffer_ptr()
{
    uint32_t* temp = tx_buffer.get_read_ptr();
    if(temp == NULL) fatal_error();
    return temp;
}

uint32_t* get_new_rx_buffer_ptr()
{
    uint32_t* temp = rx_buffer.get_write_ptr();
    if(temp == NULL) fatal_error();
    return temp;
}