#include "global.h"
#include "arm_math.h"
#include "arm_const_structs.h"

// ============================================================================
// CONFIGURATION
// ============================================================================

#define USE_SAMPLING_RATE hz96000
#define SAMPLE_RATE 96000.0f

// Echtzeit-Latenz: 500ms Analyse-Puffer
#define ANALYSIS_LATENCY_MS 500
#define ANALYSIS_DELAY_SAMPLES (96000 * ANALYSIS_LATENCY_MS / 1000)  // 48000 samples

// FFT-Parameter
#define FFT_SIZE 128              // 5.33ms @ 96kHz
#define FFT_HOP_SIZE 64          // 50% Overlap
#define FFT_FRAMES_PER_BLOCK (BLOCK_SIZE / FFT_HOP_SIZE)

// Feature-Gewichte
#define WEIGHT_CENTROID  2.0f
#define WEIGHT_RMS       3.0f
#define WEIGHT_ROLLOFF   1.5f
#define WEIGHT_FLUX      2.0f

// Clustering
#define MAX_CLUSTERS 3
#define CLUSTER_UPDATE_RATE 10    // Update Cluster-Zentren alle N Frames

// Segmentierung
#define MIN_SEGMENT_FRAMES 4      // ~10ms minimum
#define CHANGE_THRESHOLD 0.35f    // Adaptive Schwelle

// Anti-Click Fade
#define FADE_SAMPLES 192          // 2ms @ 96kHz

// Feature-Historie
#define FEATURE_HISTORY_SIZE 200  // ~5 Sekunden bei 50% Overlap

// ============================================================================
// DATA STRUCTURES
// ============================================================================

typedef struct {
    float32_t centroid;
    float32_t rms;
    float32_t rolloff;
    float32_t flux;
    float32_t zcr;
    uint32_t sample_position;     // Absolute Sample-Position
    uint8_t valid;
} FrameFeatures;

typedef struct {
    float32_t centroid;
    float32_t rms;
    float32_t rolloff;
    uint32_t sample_count;
    float32_t confidence;
} ClusterCenter;

typedef struct {
    uint32_t start_sample;
    uint32_t end_sample;
    uint8_t cluster_id;
    float32_t avg_centroid;
    float32_t avg_rms;
} Segment;

typedef struct {
    // Ring-Buffer für verzögerten Input
    int16_t input_ring[ANALYSIS_DELAY_SAMPLES];
    uint32_t input_write_pos;
    uint32_t input_read_pos;
    
    // Audio-Buffer für FFT-Analyse
    float32_t audio_buffer[FFT_SIZE];
    uint32_t audio_buffer_pos;
    
    // Feature-Historie (Ring-Buffer)
    FrameFeatures feature_history[FEATURE_HISTORY_SIZE];
    uint32_t feature_write_idx;
    uint32_t feature_count;
    
    // Spektrum-Historie für Flux
    float32_t prev_spectrum[FFT_SIZE/2];
    uint8_t prev_spectrum_valid;
    
    // Cluster-Zentren
    ClusterCenter clusters[MAX_CLUSTERS];
    uint32_t num_clusters;
    uint32_t cluster_update_counter;
    
    // Aktive Segmente (Sliding Window)
    Segment active_segments[50];
    uint32_t active_segment_count;
    uint32_t current_segment_start;
    uint8_t current_cluster_id;
    uint8_t in_segment;
    
    // Globaler Sample-Zähler
    uint32_t total_samples_processed;
    
    // Fade-State für Anti-Click
    float32_t fade_gain;
    uint8_t previous_cluster_active;
    
    // Cluster-Auswahl (via Button)
    uint8_t target_cluster;
    
    // Statistik
    uint32_t segments_detected;
    uint32_t cluster_switches;
    
} StreamingState;

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

// Streaming State
StreamingState state;

// FFT Instance
arm_rfft_fast_instance_f32 fft_instance;

// Pre-computed Hanning Window
float32_t hanning_window[FFT_SIZE];

// Performance
volatile uint32_t processing_overruns = 0;

// Button-Debouncing
uint32_t button_debounce_counter = 0;

// ============================================================================
// INITIALIZATION
// ============================================================================

void generate_hanning_window(void) {
    for(uint32_t i = 0; i < FFT_SIZE; i++) {
        hanning_window[i] = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * (float32_t)i / (float32_t)(FFT_SIZE - 1)));
    }
}

void init_streaming_state(StreamingState* s) {
    memset(s, 0, sizeof(StreamingState));
    
    // Initialisiere Cluster-Zentren mit Default-Werten
    // (werden später adaptiv angepasst)
    for(uint32_t i = 0; i < MAX_CLUSTERS; i++) {
        s->clusters[i].centroid = 1000.0f + (float32_t)i * 2000.0f;  // 1kHz, 3kHz, 5kHz
        s->clusters[i].rms = 0.1f;
        s->clusters[i].rolloff = 2000.0f + (float32_t)i * 3000.0f;
        s->clusters[i].sample_count = 0;
        s->clusters[i].confidence = 0.0f;
    }
    s->num_clusters = MAX_CLUSTERS;
    
    s->target_cluster = 0;  // Start mit Cluster 0
    s->fade_gain = 0.0f;
    s->previous_cluster_active = 0;
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static inline float32_t fast_normalize(int16_t x) {
    return (float32_t)x * 0.000030518f;  // 1/32768
}

static inline int16_t float_to_int16(float32_t x) {
    if(x > 1.0f) x = 1.0f;
    if(x < -1.0f) x = -1.0f;
    return (int16_t)(x * 32767.0f);
}

// Ring-Buffer Zugriff
static inline void ring_write(int16_t* ring, uint32_t size, uint32_t* pos, int16_t value) {
    ring[*pos] = value;
    *pos = (*pos + 1) % size;
}

static inline int16_t ring_read(int16_t* ring, uint32_t size, uint32_t* pos) {
    int16_t value = ring[*pos];
    *pos = (*pos + 1) % size;
    return value;
}

// ============================================================================
// FEATURE EXTRACTION
// ============================================================================

void extract_features(float32_t* audio_frame, FrameFeatures* features, 
                     float32_t* prev_spec, uint32_t sample_pos) {
    
    // 1. RMS
    float32_t rms;
    arm_rms_f32(audio_frame, FFT_SIZE, &rms);
    features->rms = rms;
    
    // Schwellwert: Stille erkennen
    if(rms < 0.005f) {
        features->valid = 0;
        features->centroid = 0.0f;
        features->rolloff = 0.0f;
        features->flux = 0.0f;
        features->zcr = 0.0f;
        features->sample_position = sample_pos;
        return;
    }
    
    features->valid = 1;
    features->sample_position = sample_pos;
    
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
    
    // 6. Spectral Centroid & Total Power
    float32_t weighted_sum = 0.0f;
    float32_t total_power = 0.0f;
    float32_t freq_resolution = SAMPLE_RATE / (float32_t)FFT_SIZE;
    
    for(uint32_t i = 1; i < FFT_SIZE/2; i++) {  // Skip DC
        float32_t power = spectrum_magnitude[i] * spectrum_magnitude[i];
        float32_t freq = (float32_t)i * freq_resolution;
        weighted_sum += freq * power;
        total_power += power;
    }
    
    features->centroid = (total_power > 1e-10f) ? (weighted_sum / total_power) : 0.0f;
    
    // 7. Spectral Rolloff (85%)
    if(total_power > 1e-10f) {
        float32_t cumsum = 0.0f;
        float32_t threshold = 0.85f * total_power;
        
        for(uint32_t i = 1; i < FFT_SIZE/2; i++) {
            cumsum += spectrum_magnitude[i] * spectrum_magnitude[i];
            if(cumsum >= threshold) {
                features->rolloff = (float32_t)i * freq_resolution;
                break;
            }
        }
    } else {
        features->rolloff = 0.0f;
    }
    
    // 8. Spectral Flux
    if(prev_spec != NULL) {
        float32_t flux = 0.0f;
        for(uint32_t i = 1; i < FFT_SIZE/2; i++) {
            float32_t diff = spectrum_magnitude[i] - prev_spec[i];
            if(diff > 0.0f) {  // Nur positive Unterschiede (Half-Wave Rectification)
                flux += diff * diff;
            }
        }
        features->flux = flux;
        
        // Update prev_spectrum
        arm_copy_f32(spectrum_magnitude, prev_spec, FFT_SIZE/2);
    } else {
        features->flux = 0.0f;
    }
}

// ============================================================================
// CLUSTERING (Online K-Means)
// ============================================================================

float32_t compute_cluster_distance(FrameFeatures* feat, ClusterCenter* cluster) {
    // Normalisierte euklidische Distanz
    float32_t d_cent = (feat->centroid - cluster->centroid) / 10000.0f;  // Norm auf ~1
    float32_t d_rms = (feat->rms - cluster->rms) * 5.0f;                // Norm auf ~1
    float32_t d_roll = (feat->rolloff - cluster->rolloff) / 10000.0f;   // Norm auf ~1
    
    float32_t dist = WEIGHT_CENTROID * d_cent * d_cent +
                     WEIGHT_RMS * d_rms * d_rms +
                     WEIGHT_ROLLOFF * d_roll * d_roll;
    
    return dist;
}

uint8_t assign_to_cluster(StreamingState* s, FrameFeatures* feat) {
    if(!feat->valid) {
        return s->current_cluster_id;  // Bei Stille: behalte aktuelles Cluster
    }
    
    // Finde nächstes Cluster
    float32_t min_dist = 1e10f;
    uint8_t best_cluster = 0;
    
    for(uint32_t i = 0; i < s->num_clusters; i++) {
        float32_t dist = compute_cluster_distance(feat, &s->clusters[i]);
        if(dist < min_dist) {
            min_dist = dist;
            best_cluster = i;
        }
    }
    
    return best_cluster;
}

void update_cluster_centers(StreamingState* s) {
    // Sammle Features der letzten N Frames pro Cluster
    typedef struct {
        float32_t sum_centroid;
        float32_t sum_rms;
        float32_t sum_rolloff;
        uint32_t count;
    } ClusterAccumulator;
    
    ClusterAccumulator acc[MAX_CLUSTERS];
    memset(acc, 0, sizeof(acc));
    
    // Durchlaufe Feature-Historie
    uint32_t history_size = (s->feature_count < FEATURE_HISTORY_SIZE) ? 
                            s->feature_count : FEATURE_HISTORY_SIZE;
    
    for(uint32_t i = 0; i < history_size; i++) {
        uint32_t idx = (s->feature_write_idx + FEATURE_HISTORY_SIZE - history_size + i) % FEATURE_HISTORY_SIZE;
        FrameFeatures* feat = &s->feature_history[idx];
        
        if(feat->valid) {
            uint8_t cluster = assign_to_cluster(s, feat);
            acc[cluster].sum_centroid += feat->centroid;
            acc[cluster].sum_rms += feat->rms;
            acc[cluster].sum_rolloff += feat->rolloff;
            acc[cluster].count++;
        }
    }
    
    // Update Cluster-Zentren mit gleitendem Durchschnitt
    const float32_t alpha = 0.1f;  // Lernrate
    
    for(uint32_t i = 0; i < s->num_clusters; i++) {
        if(acc[i].count > 5) {  // Mindestens 5 Samples
            float32_t new_centroid = acc[i].sum_centroid / (float32_t)acc[i].count;
            float32_t new_rms = acc[i].sum_rms / (float32_t)acc[i].count;
            float32_t new_rolloff = acc[i].sum_rolloff / (float32_t)acc[i].count;
            
            s->clusters[i].centroid = (1.0f - alpha) * s->clusters[i].centroid + alpha * new_centroid;
            s->clusters[i].rms = (1.0f - alpha) * s->clusters[i].rms + alpha * new_rms;
            s->clusters[i].rolloff = (1.0f - alpha) * s->clusters[i].rolloff + alpha * new_rolloff;
            s->clusters[i].sample_count = acc[i].count;
            s->clusters[i].confidence = (float32_t)acc[i].count / (float32_t)history_size;
        }
    }
}

// ============================================================================
// CHANGE DETECTION & SEGMENTATION
// ============================================================================

float32_t compute_feature_change(FrameFeatures* current, FrameFeatures* previous) {
    if(!current->valid || !previous->valid) {
        return 0.0f;
    }
    
    // Normalisierte Änderungen
    float32_t d_cent = fabsf(current->centroid - previous->centroid) / 10000.0f;
    float32_t d_rms = fabsf(current->rms - previous->rms) * 5.0f;
    float32_t d_rolloff = fabsf(current->rolloff - previous->rolloff) / 10000.0f;
    float32_t d_flux = current->flux * 0.01f;  // Flux ist bereits ein Change-Maß
    
    // Clipping
    if(d_cent > 1.0f) d_cent = 1.0f;
    if(d_rms > 1.0f) d_rms = 1.0f;
    if(d_rolloff > 1.0f) d_rolloff = 1.0f;
    if(d_flux > 1.0f) d_flux = 1.0f;
    
    // Gewichtete Kombination
    float32_t total_weight = WEIGHT_CENTROID + WEIGHT_RMS + WEIGHT_ROLLOFF + WEIGHT_FLUX;
    float32_t change = (WEIGHT_CENTROID * d_cent +
                       WEIGHT_RMS * d_rms +
                       WEIGHT_ROLLOFF * d_rolloff +
                       WEIGHT_FLUX * d_flux) / total_weight;
    
    return change;
}

void detect_segment_change(StreamingState* s, FrameFeatures* current_feat) {
    // Hole vorheriges Feature
    if(s->feature_count < 2) {
        s->in_segment = 1;
        s->current_segment_start = current_feat->sample_position;
        s->current_cluster_id = assign_to_cluster(s, current_feat);
        return;
    }
    
    uint32_t prev_idx = (s->feature_write_idx + FEATURE_HISTORY_SIZE - 1) % FEATURE_HISTORY_SIZE;
    FrameFeatures* prev_feat = &s->feature_history[prev_idx];
    
    // Berechne Change Score
    float32_t change = compute_feature_change(current_feat, prev_feat);
    
    // Cluster-Zuordnung
    uint8_t new_cluster = assign_to_cluster(s, current_feat);
    
    // Segment-Wechsel erkennen
    uint8_t cluster_changed = (new_cluster != s->current_cluster_id);
    uint8_t significant_change = (change > CHANGE_THRESHOLD);
    
    if((cluster_changed || significant_change) && s->in_segment) {
        // Minimale Segment-Länge prüfen
        uint32_t segment_length = current_feat->sample_position - s->current_segment_start;
        
        if(segment_length >= FFT_HOP_SIZE * MIN_SEGMENT_FRAMES) {
            // Altes Segment abschließen
            if(s->active_segment_count < 50) {
                Segment* seg = &s->active_segments[s->active_segment_count];
                seg->start_sample = s->current_segment_start;
                seg->end_sample = current_feat->sample_position;
                seg->cluster_id = s->current_cluster_id;
                s->active_segment_count++;
                
                // Entferne alte Segmente (behalte nur letzte 50)
                if(s->active_segment_count >= 50) {
                    memmove(s->active_segments, &s->active_segments[1], 
                           49 * sizeof(Segment));
                    s->active_segment_count = 49;
                }
            }
            
            // Neues Segment starten
            s->current_segment_start = current_feat->sample_position;
            s->current_cluster_id = new_cluster;
            s->segments_detected++;
            s->cluster_switches++;
        }
    }
}

// ============================================================================
// PLAYBACK WITH ANTI-CLICK
// ============================================================================

void process_playback_sample(StreamingState* s, int16_t input_sample, 
                             int16_t* output_sample, uint32_t global_pos) {
    
    // Finde Cluster für aktuelle Position (global_pos - ANALYSIS_DELAY)
    uint32_t analyzed_pos = (global_pos >= ANALYSIS_DELAY_SAMPLES) ? 
                            (global_pos - ANALYSIS_DELAY_SAMPLES) : 0;
    
    // Suche Segment in aktiven Segmenten
    uint8_t target_cluster = s->target_cluster;
    uint8_t should_play = 0;
    
    for(uint32_t i = 0; i < s->active_segment_count; i++) {
        Segment* seg = &s->active_segments[i];
        if(analyzed_pos >= seg->start_sample && analyzed_pos < seg->end_sample) {
            if(seg->cluster_id == target_cluster) {
                should_play = 1;
            }
            break;
        }
    }
    
    // Anti-Click Fade
    const float32_t fade_speed = 1.0f / (float32_t)FADE_SAMPLES;
    
    if(should_play) {
        // Fade-In
        if(s->fade_gain < 1.0f) {
            s->fade_gain += fade_speed;
            if(s->fade_gain > 1.0f) s->fade_gain = 1.0f;
        }
    } else {
        // Fade-Out
        if(s->fade_gain > 0.0f) {
            s->fade_gain -= fade_speed;
            if(s->fade_gain < 0.0f) s->fade_gain = 0.0f;
        }
    }
    
    // Wende Gain an
    float32_t output_f = (float32_t)input_sample * s->fade_gain;
    *output_sample = float_to_int16(output_f * 0.000030518f);
    
    s->previous_cluster_active = should_play;
}

// ============================================================================
// MAIN PROCESSING LOOP
// ============================================================================

int main()
{
    init_platform(115200, USE_SAMPLING_RATE, line_in);
    
    debug_printf("%s, %s\n", __DATE__, __TIME__);
    IF_DEBUG(debug_printf("=== REAL-TIME AUDIO DEINTERLEAVING ===\n"));
    IF_DEBUG(debug_printf("Sample Rate: 96kHz\n"));
    IF_DEBUG(debug_printf("Analysis Latency: %d ms\n", ANALYSIS_LATENCY_MS));
    IF_DEBUG(debug_printf("FFT Size: %d (%.2f ms)\n", FFT_SIZE, 
             (float)FFT_SIZE * 1000.0f / 96000.0f));
    
    gpio_set(TEST_PIN, LOW);
    
    rx_buffer.init();
    tx_buffer.init();
    memset(in, 0, sizeof(in));
    memset(out, 0, sizeof(out));
    
    // FFT Init
    arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);
    generate_hanning_window();
    
    // State Init
    init_streaming_state(&state);

    state.target_cluster = 0;
    
    platform_start();
    
    IF_DEBUG(debug_printf("Fixed to cluster 0 (hardcoded)\n"));
    
    while(true)
    {
        if(!rx_buffer.read(in)) {
            continue;
        }
        
        gpio_set(LED_B, HIGH);
        gpio_set(TEST_PIN, HIGH);
        
        convert_audio_sample_to_2ch(in, left_in, right_in);
        
        // ========== HAUPTVERARBEITUNG ==========
        
        for(uint32_t n = 0; n < BLOCK_SIZE; n++) {
            
            // 1. SCHREIBE INPUT IN RING-BUFFER
            ring_write(state.input_ring, ANALYSIS_DELAY_SAMPLES, 
                      &state.input_write_pos, left_in[n]);
            
            // 2. SAMMLE SAMPLES FÜR FFT
            if(state.audio_buffer_pos < FFT_SIZE) {
                state.audio_buffer[state.audio_buffer_pos++] = fast_normalize(left_in[n]);
            }
            
            // 3. FFT-ANALYSE (wenn Buffer voll)
            if(state.audio_buffer_pos >= FFT_SIZE) {
                
                FrameFeatures current_features;
                
                extract_features(state.audio_buffer, &current_features,
                               state.prev_spectrum_valid ? state.prev_spectrum : NULL,
                               state.total_samples_processed - FFT_SIZE + FFT_HOP_SIZE);
                
                state.prev_spectrum_valid = 1;
                
                // Speichere in Feature-Historie
                state.feature_history[state.feature_write_idx] = current_features;
                state.feature_write_idx = (state.feature_write_idx + 1) % FEATURE_HISTORY_SIZE;
                if(state.feature_count < FEATURE_HISTORY_SIZE) state.feature_count++;
                
                // Segment-Detektion
                detect_segment_change(&state, &current_features);
                
                // Cluster-Update (periodisch)
                if(++state.cluster_update_counter >= CLUSTER_UPDATE_RATE) {
                    update_cluster_centers(&state);
                    state.cluster_update_counter = 0;
                }
                
                // Schiebe Audio-Buffer (Overlap)
                memmove(state.audio_buffer, &state.audio_buffer[FFT_HOP_SIZE],
                       (FFT_SIZE - FFT_HOP_SIZE) * sizeof(float32_t));
                state.audio_buffer_pos = FFT_SIZE - FFT_HOP_SIZE;
            }
            
            // 4. LESE VERZÖGERTEN INPUT & GENERIERE OUTPUT
            int16_t delayed_sample = ring_read(state.input_ring, ANALYSIS_DELAY_SAMPLES,
                                              &state.input_read_pos);
            
            process_playback_sample(&state, delayed_sample, &left_out[n],
                                   state.total_samples_processed);
            
            right_out[n] = left_out[n];  // Mono → Stereo
            
            state.total_samples_processed++;
        }
        
        // Output
        convert_2ch_to_audio_sample(left_out, right_out, out);
        
        if(!tx_buffer.write(out)) {
            processing_overruns++;
        }
        
        gpio_set(LED_B, LOW);
        gpio_set(TEST_PIN, LOW);
        
        // Status Output (alle 2 Sekunden)
        #ifdef DEBUG
        static uint32_t status_counter = 0;
        if(++status_counter >= 96000 / BLOCK_SIZE * 2) {
            debug_printf("Time: %.1fs | Segments: %lu | Cluster: %d | Overruns: %lu\n",
                        (float)state.total_samples_processed / 96000.0f,
                        state.segments_detected,
                        state.target_cluster,
                        processing_overruns);
            
            // Cluster-Statistik
            debug_printf("Clusters: ");
            for(uint32_t i = 0; i < state.num_clusters; i++) {
                debug_printf("C%lu(%.0fHz,%.0f%%) ", i, 
                           state.clusters[i].centroid,
                           state.clusters[i].confidence * 100.0f);
            }
            debug_printf("\n");
            
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