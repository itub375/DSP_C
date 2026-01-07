#include "global.h"
#include "arm_math.h"
#include "arm_const_structs.h"

// ============================================================================
// CONFIGURATION
// ============================================================================

#define USE_SAMPLING_RATE hz96000
#define SAMPLE_RATE 96000.0f

#define ANALYSIS_LATENCY_MS 200
#define ANALYSIS_DELAY_SAMPLES (96000 * ANALYSIS_LATENCY_MS / 1000)

#define FFT_SIZE 256
#define FFT_HOP_SIZE 128

// Feature Weights
#define WEIGHT_CENTROID  2.0f
#define WEIGHT_RMS       3.0f
#define WEIGHT_ROLLOFF   1.5f
#define WEIGHT_FLUX      2.0f

// Clustering
#define MAX_CLUSTERS 3
#define CLUSTER_UPDATE_RATE 10

// Segmentation
#define MIN_SEGMENT_FRAMES 4
#define CHANGE_THRESHOLD 0.35f

// Anti-Click
#define FADE_SAMPLES 192

// Feature History
#define FEATURE_HISTORY_SIZE 100

// ============================================================================
// NEUE KOMPONENTEN FÜR ECHTES DEINTERLEAVING
// ============================================================================

// Segment Buffer (maximal 500ms pro Segment bei 96kHz)
#define MAX_SEGMENT_SAMPLES 48000
#define MAX_STORED_SEGMENTS 20

// Rekonstruktions-Buffer (5 Sekunden)
#define RECONSTRUCTED_BUFFER_SIZE (SAMPLE_RATE * 5)

// ============================================================================
// DATA STRUCTURES
// ============================================================================

typedef struct {
    float32_t centroid;
    float32_t rms;
    float32_t rolloff;
    float32_t flux;
    float32_t zcr;
    uint32_t sample_position;
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

// NEU: Gespeichertes Segment mit Audio-Daten
typedef struct {
    int16_t samples[MAX_SEGMENT_SAMPLES];
    uint32_t length;
    uint32_t original_position;
    uint8_t cluster_id;
    uint8_t valid;
} StoredSegment;

// NEU: Segment-Queue für Speicherung
typedef struct {
    StoredSegment segments[MAX_STORED_SEGMENTS];
    uint32_t write_idx;
    uint32_t count;
} SegmentQueue;

// NEU: Rekonstruktions-Buffer
typedef struct {
    int16_t data[RECONSTRUCTED_BUFFER_SIZE];
    uint32_t write_pos;
    uint32_t read_pos;
    uint32_t available_samples;
    uint8_t buffer_ready;
} ReconstructedBuffer;

typedef struct {
    // Ring-Buffer für verzögerten Input
    int16_t input_ring[ANALYSIS_DELAY_SAMPLES];
    uint32_t input_write_pos;
    uint32_t input_read_pos;
    
    // Audio-Buffer für FFT
    float32_t audio_buffer[FFT_SIZE];
    uint32_t audio_buffer_pos;
    
    // Feature History
    FrameFeatures feature_history[FEATURE_HISTORY_SIZE];
    uint32_t feature_write_idx;
    uint32_t feature_count;
    
    // Spektrum
    float32_t prev_spectrum[FFT_SIZE/2];
    uint8_t prev_spectrum_valid;
    
    // Cluster
    ClusterCenter clusters[MAX_CLUSTERS];
    uint32_t num_clusters;
    uint32_t cluster_update_counter;
    
    // Segmente (aktive Erkennung)
    Segment active_segments[50];
    uint32_t active_segment_count;
    uint32_t current_segment_start;
    uint8_t current_cluster_id;
    uint8_t in_segment;
    
    // NEU: Segment-Speicher
    SegmentQueue segment_queue;
    
    // NEU: Rekonstruktions-Buffer
    ReconstructedBuffer recon_buffer;
    
    // Sample Counter
    uint32_t total_samples_processed;
    
    // Fade & Control
    float32_t fade_gain;
    uint8_t previous_cluster_active;
    uint8_t target_cluster;
    
    // Statistik
    uint32_t segments_detected;
    uint32_t cluster_switches;
    
} StreamingState;

// ============================================================================
// GLOBALS
// ============================================================================

CircularBuffer rx_buffer;
CircularBuffer tx_buffer;

uint32_t in[BLOCK_SIZE];
uint32_t out[BLOCK_SIZE];
int16_t left_in[BLOCK_SIZE];
int16_t right_in[BLOCK_SIZE];
int16_t left_out[BLOCK_SIZE];
int16_t right_out[BLOCK_SIZE];

uint32_t dummy_zeros[BLOCK_SIZE];

float32_t fft_input[FFT_SIZE];
float32_t fft_output[FFT_SIZE * 2];
float32_t spectrum_magnitude[FFT_SIZE/2];

StreamingState state;
arm_rfft_fast_instance_f32 fft_instance;
float32_t hanning_window[FFT_SIZE];

volatile uint32_t processing_overruns = 0;
volatile uint32_t buffer_underruns = 0;

// Button Debounce
uint32_t last_button_time = 0;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

void generate_hanning_window(void) {
    for(uint32_t i = 0; i < FFT_SIZE; i++) {
        hanning_window[i] = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * (float32_t)i / (float32_t)(FFT_SIZE - 1)));
    }
}

void init_streaming_state(StreamingState* s) {
    memset(s, 0, sizeof(StreamingState));
    
    for(uint32_t i = 0; i < MAX_CLUSTERS; i++) {
        s->clusters[i].centroid = 1000.0f + (float32_t)i * 2000.0f;
        s->clusters[i].rms = 0.1f;
        s->clusters[i].rolloff = 2000.0f + (float32_t)i * 3000.0f;
    }
    s->num_clusters = MAX_CLUSTERS;
    s->target_cluster = 0;
}

static inline float32_t fast_normalize(int16_t x) {
    return (float32_t)x * 0.000030518f;
}

static inline int16_t float_to_int16(float32_t x) {
    if(x > 1.0f) x = 1.0f;
    else if(x < -1.0f) x = -1.0f;
    return (int16_t)(x * 32767.0f);
}

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
// DSP LOGIC (Feature Extraction + Clustering wie vorher)
// ============================================================================

void extract_features(float32_t* audio_frame, FrameFeatures* features, float32_t* prev_spec, uint32_t sample_pos) {
    float32_t rms;
    arm_rms_f32(audio_frame, FFT_SIZE, &rms);
    features->rms = rms;
    
    if(rms < 0.005f) {
        features->valid = 0;
        features->centroid = 0; features->rolloff = 0; features->flux = 0; features->zcr = 0;
        features->sample_position = sample_pos;
        return;
    }
    
    features->valid = 1;
    features->sample_position = sample_pos;
    
    // ZCR
    uint32_t zero_crossings = 0;
    for(uint32_t i = 1; i < FFT_SIZE; i++) {
        if((audio_frame[i-1] >= 0.0f && audio_frame[i] < 0.0f) || 
           (audio_frame[i-1] < 0.0f && audio_frame[i] >= 0.0f)) zero_crossings++;
    }
    features->zcr = (float32_t)zero_crossings / (float32_t)FFT_SIZE;
    
    arm_mult_f32(audio_frame, hanning_window, fft_input, FFT_SIZE);
    arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0);
    arm_cmplx_mag_f32(fft_output, spectrum_magnitude, FFT_SIZE/2);
    
    float32_t weighted_sum = 0.0f;
    float32_t total_power = 0.0f;
    float32_t freq_resolution = SAMPLE_RATE / (float32_t)FFT_SIZE;
    
    for(uint32_t i = 1; i < FFT_SIZE/2; i++) {
        float32_t power = spectrum_magnitude[i] * spectrum_magnitude[i];
        weighted_sum += (float32_t)i * freq_resolution * power;
        total_power += power;
    }
    features->centroid = (total_power > 1e-10f) ? (weighted_sum / total_power) : 0.0f;
    
    // Rolloff
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
    } else { features->rolloff = 0.0f; }
    
    // Flux
    if(prev_spec != NULL) {
        float32_t flux = 0.0f;
        for(uint32_t i = 1; i < FFT_SIZE/2; i++) {
            float32_t diff = spectrum_magnitude[i] - prev_spec[i];
            if(diff > 0.0f) flux += diff * diff;
        }
        features->flux = flux;
        arm_copy_f32(spectrum_magnitude, prev_spec, FFT_SIZE/2);
    } else { features->flux = 0.0f; }
}

float32_t compute_cluster_distance(FrameFeatures* feat, ClusterCenter* cluster) {
    float32_t d_cent = (feat->centroid - cluster->centroid) / 10000.0f;
    float32_t d_rms = (feat->rms - cluster->rms) * 5.0f;
    float32_t d_roll = (feat->rolloff - cluster->rolloff) / 10000.0f;
    return WEIGHT_CENTROID * d_cent * d_cent + WEIGHT_RMS * d_rms * d_rms + WEIGHT_ROLLOFF * d_roll * d_roll;
}

uint8_t assign_to_cluster(StreamingState* s, FrameFeatures* feat) {
    if(!feat->valid) return s->current_cluster_id;
    float32_t min_dist = 1e10f; 
    uint8_t best = 0;
    for(uint32_t i = 0; i < s->num_clusters; i++) {
        float32_t dist = compute_cluster_distance(feat, &s->clusters[i]);
        if(dist < min_dist) { min_dist = dist; best = i; }
    }
    return best;
}

void update_cluster_centers(StreamingState* s) {
    for(uint32_t c = 0; c < s->num_clusters; c++) {
        float32_t sum_cent = 0, sum_rms = 0, sum_roll = 0;
        uint32_t count = 0;
        for(uint32_t i = 0; i < s->feature_count; i++) {
            FrameFeatures* f = &s->feature_history[i];
            if(!f->valid) continue;
            if(assign_to_cluster(s, f) == c) {
                sum_cent += f->centroid;
                sum_rms += f->rms;
                sum_roll += f->rolloff;
                count++;
            }
        }
        if(count > 0) {
            const float32_t alpha = 0.1f;
            s->clusters[c].centroid = (1.0f - alpha) * s->clusters[c].centroid + alpha * (sum_cent / count);
            s->clusters[c].rms = (1.0f - alpha) * s->clusters[c].rms + alpha * (sum_rms / count);
            s->clusters[c].rolloff = (1.0f - alpha) * s->clusters[c].rolloff + alpha * (sum_roll / count);
        }
    }
}

void detect_segment_change(StreamingState* s, FrameFeatures* current_feat) {
    uint8_t new_cluster = assign_to_cluster(s, current_feat);
    
    if(!s->in_segment) {
        if(current_feat->valid && current_feat->rms > 0.01f) {
            s->in_segment = 1;
            s->current_segment_start = current_feat->sample_position;
            s->current_cluster_id = new_cluster;
        }
    } else {
        if(new_cluster != s->current_cluster_id || current_feat->rms < 0.005f) {
            uint32_t seg_len = current_feat->sample_position - s->current_segment_start;
            if(seg_len > (MIN_SEGMENT_FRAMES * FFT_HOP_SIZE)) {
                if(s->active_segment_count < 50) {
                    Segment* seg = &s->active_segments[s->active_segment_count++];
                    seg->start_sample = s->current_segment_start;
                    seg->end_sample = current_feat->sample_position;
                    seg->cluster_id = s->current_cluster_id;
                    s->segments_detected++;
                }
            }
            s->in_segment = (current_feat->rms > 0.01f);
            s->current_segment_start = current_feat->sample_position;
            s->current_cluster_id = new_cluster;
        }
    }
}

// ============================================================================
// NEUE FUNKTIONEN: SEGMENT-EXTRAKTION & REKONSTRUKTION
// ============================================================================

// Extrahiere Segment aus delay_ring und speichere in Queue
void extract_and_store_segment(StreamingState* s, Segment* seg) {
    if(s->segment_queue.count >= MAX_STORED_SEGMENTS) {
        // Queue voll, ältestes überschreiben
        s->segment_queue.write_idx = 0;
        s->segment_queue.count = MAX_STORED_SEGMENTS - 1;
    }
    
    StoredSegment* stored = &s->segment_queue.segments[s->segment_queue.write_idx];
    
    uint32_t seg_length = seg->end_sample - seg->start_sample;
    if(seg_length > MAX_SEGMENT_SAMPLES) seg_length = MAX_SEGMENT_SAMPLES;
    
    // Berechne Startposition im Ring-Buffer
    uint32_t current_pos = s->input_write_pos;
    uint32_t delay = ANALYSIS_DELAY_SAMPLES;
    uint32_t samples_back = s->total_samples_processed - seg->start_sample;
    
    if(samples_back < delay) {
        uint32_t read_pos = (current_pos + delay - samples_back) % delay;
        
        // Kopiere Samples aus Ring-Buffer
        for(uint32_t i = 0; i < seg_length; i++) {
            stored->samples[i] = s->input_ring[read_pos];
            read_pos = (read_pos + 1) % delay;
        }
        
        stored->length = seg_length;
        stored->original_position = seg->start_sample;
        stored->cluster_id = seg->cluster_id;
        stored->valid = 1;
        
        s->segment_queue.write_idx = (s->segment_queue.write_idx + 1) % MAX_STORED_SEGMENTS;
        if(s->segment_queue.count < MAX_STORED_SEGMENTS) s->segment_queue.count++;
    }
}

// Finde alle Segmente eines Clusters und fülle Rekonstruktions-Buffer
void fill_reconstructed_buffer(StreamingState* s, uint8_t target_cluster) {
    if(s->recon_buffer.buffer_ready) return; // Buffer bereits gefüllt
    if(s->segment_queue.count < 3) return; // Zu wenig Segmente
    
    // Sortiere Segmente nach Original-Position (Bubble-Sort, einfach)
    for(uint32_t i = 0; i < s->segment_queue.count - 1; i++) {
        for(uint32_t j = 0; j < s->segment_queue.count - i - 1; j++) {
            if(s->segment_queue.segments[j].original_position > 
               s->segment_queue.segments[j+1].original_position) {
                StoredSegment temp = s->segment_queue.segments[j];
                s->segment_queue.segments[j] = s->segment_queue.segments[j+1];
                s->segment_queue.segments[j+1] = temp;
            }
        }
    }
    
    // Kopiere alle Segmente des Ziel-Clusters in den Rekonstruktions-Buffer
    uint32_t write_pos = 0;
    for(uint32_t i = 0; i < s->segment_queue.count; i++) {
        StoredSegment* seg = &s->segment_queue.segments[i];
        if(seg->valid && seg->cluster_id == target_cluster) {
            // Kopiere Segment mit kurzen Fades
            uint32_t fade_len = (FADE_SAMPLES < seg->length / 4) ? FADE_SAMPLES : seg->length / 4;
            
            for(uint32_t j = 0; j < seg->length; j++) {
                if(write_pos >= RECONSTRUCTED_BUFFER_SIZE) break;
                
                float32_t fade = 1.0f;
                if(j < fade_len) fade = (float32_t)j / (float32_t)fade_len; // Fade-In
                if(j > seg->length - fade_len) fade = (float32_t)(seg->length - j) / (float32_t)fade_len; // Fade-Out
                
                s->recon_buffer.data[write_pos++] = (int16_t)(seg->samples[j] * fade);
            }
        }
    }
    
    s->recon_buffer.write_pos = write_pos;
    s->recon_buffer.read_pos = 0;
    s->recon_buffer.available_samples = write_pos;
    s->recon_buffer.buffer_ready = (write_pos > BLOCK_SIZE * 2);
}

// Lese Sample aus Rekonstruktions-Buffer
int16_t read_from_reconstructed_buffer(StreamingState* s) {
    if(!s->recon_buffer.buffer_ready || s->recon_buffer.available_samples == 0) {
        return 0; // Stille wenn Buffer leer
    }
    
    int16_t sample = s->recon_buffer.data[s->recon_buffer.read_pos];
    s->recon_buffer.read_pos++;
    s->recon_buffer.available_samples--;
    
    // Wenn Buffer fast leer, neu füllen
    if(s->recon_buffer.available_samples < BLOCK_SIZE) {
        s->recon_buffer.buffer_ready = 0;
        fill_reconstructed_buffer(s, s->target_cluster);
    }
    
    return sample;
}

// ============================================================================
// BUTTON HANDLER
// ============================================================================

void handle_button_press(StreamingState* s) {
    if(gpio_get(USER_BUTTON) == 0) { // Button gedrückt (active low)
        uint32_t now = s->total_samples_processed / (SAMPLE_RATE / 1000); // Approx. ms
        
        if((now - last_button_time) > 300) { // 300ms Debounce
            s->target_cluster = (s->target_cluster + 1) % s->num_clusters;
            s->recon_buffer.buffer_ready = 0; // Buffer neu füllen
            last_button_time = now;
            
            // LED-Feedback
            gpio_set(LED_G, s->target_cluster == 0 ? LOW : HIGH);
            gpio_set(LED_R, s->target_cluster == 1 ? LOW : HIGH);
        }
    }
}

// ============================================================================
// MAIN
// ============================================================================

int main()
{
    init_platform(115200, USE_SAMPLING_RATE, line_in);
    
    rx_buffer.init();
    tx_buffer.init();
    memset(in, 0, sizeof(in));
    memset(out, 0, sizeof(out));
    memset(dummy_zeros, 0, sizeof(dummy_zeros));
    
    arm_status status = arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);
    if (status != ARM_MATH_SUCCESS) {
        fatal_error();
    }
    generate_hanning_window();
    
    init_streaming_state(&state);
    state.target_cluster = 0;
    
    // Buffer pre-filling
    memset(out, 0, sizeof(out));
    tx_buffer.write(out);
    tx_buffer.write(out);
    
    platform_start();
    
    while(true)
    {
        if(!rx_buffer.read(in)) {
            continue; 
        }
        
        gpio_set(LED_B, HIGH);
        convert_audio_sample_to_2ch(in, left_in, right_in);
        
        // Process Block
        for(uint32_t n = 0; n < BLOCK_SIZE; n++) {
            ring_write(state.input_ring, ANALYSIS_DELAY_SAMPLES, &state.input_write_pos, left_in[n]);
            
            if(state.audio_buffer_pos < FFT_SIZE) {
                state.audio_buffer[state.audio_buffer_pos++] = fast_normalize(left_in[n]);
            }
            
            if(state.audio_buffer_pos >= FFT_SIZE) {
                FrameFeatures current_features;
                extract_features(state.audio_buffer, &current_features,
                               state.prev_spectrum_valid ? state.prev_spectrum : NULL,
                               state.total_samples_processed - FFT_SIZE + FFT_HOP_SIZE);
                
                state.prev_spectrum_valid = 1;
                state.feature_history[state.feature_write_idx] = current_features;
                state.feature_write_idx = (state.feature_write_idx + 1) % FEATURE_HISTORY_SIZE;
                if(state.feature_count < FEATURE_HISTORY_SIZE) state.feature_count++;
                
                detect_segment_change(&state, &current_features);
                
                if(++state.cluster_update_counter >= CLUSTER_UPDATE_RATE) {
                    update_cluster_centers(&state);
                    state.cluster_update_counter = 0;
                }
                
                memmove(state.audio_buffer, &state.audio_buffer[FFT_HOP_SIZE], 
                       (FFT_SIZE - FFT_HOP_SIZE) * sizeof(float32_t));
                state.audio_buffer_pos = FFT_SIZE - FFT_HOP_SIZE;
            }
            
            // NEU: Segmente extrahieren wenn komplett
            for(uint32_t i = 0; i < state.active_segment_count; i++) {
                Segment* seg = &state.active_segments[i];
                if(state.total_samples_processed > seg->end_sample + ANALYSIS_DELAY_SAMPLES) {
                    extract_and_store_segment(&state, seg);
                    // Segment aus aktiver Liste entfernen
                    for(uint32_t j = i; j < state.active_segment_count - 1; j++) {
                        state.active_segments[j] = state.active_segments[j+1];
                    }
                    state.active_segment_count--;
                    i--;
                }
            }
            
            // NEU: Output aus rekonstruiertem Buffer
            left_out[n] = read_from_reconstructed_buffer(&state);
            right_out[n] = left_out[n];
            
            state.total_samples_processed++;
        }
        
        // Button Check
        handle_button_press(&state);
        
        // Output
        convert_2ch_to_audio_sample(left_out, right_out, out);
        
        if(!tx_buffer.write(out)) {
            processing_overruns++;
        }
        
        gpio_set(LED_B, LOW);
    }
    
    return 0;
}

// ============================================================================
// CALLBACKS
// ============================================================================

uint32_t* get_new_tx_buffer_ptr()
{
    uint32_t* temp = tx_buffer.get_read_ptr();
    if(temp == NULL) {
        buffer_underruns++;
        return dummy_zeros; 
    }
    return temp;
}

uint32_t* get_new_rx_buffer_ptr()
{
    uint32_t* temp = rx_buffer.get_write_ptr();
    if(temp == NULL) {
        return dummy_zeros; 
    }
    return temp;
}