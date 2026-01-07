#include "global.h"
#include "arm_math.h"
#include "arm_const_structs.h"

// ============================================================================
// CONFIGURATION
// ============================================================================

#define USE_SAMPLING_RATE hz96000
#define SAMPLE_RATE 96000.0f

// Latenz reduziert, um Speicher zu sparen und Start zu beschleunigen
#define ANALYSIS_LATENCY_MS 200
#define ANALYSIS_DELAY_SAMPLES (96000 * ANALYSIS_LATENCY_MS / 1000)

// FFT-Parameter
#define FFT_SIZE 256
#define FFT_HOP_SIZE 128
#define FFT_FRAMES_PER_BLOCK (BLOCK_SIZE / FFT_HOP_SIZE)

// Feature-Gewichte
#define WEIGHT_CENTROID  2.0f
#define WEIGHT_RMS       3.0f
#define WEIGHT_ROLLOFF   1.5f
#define WEIGHT_FLUX      2.0f

// Clustering
#define MAX_CLUSTERS 3
#define CLUSTER_UPDATE_RATE 10

// Segmentierung
#define MIN_SEGMENT_FRAMES 4
#define CHANGE_THRESHOLD 0.35f

// Anti-Click Fade
#define FADE_SAMPLES 192

// Feature-Historie
#define FEATURE_HISTORY_SIZE 100

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

typedef struct {
    // Ring-Buffer für verzögerten Input
    int16_t input_ring[ANALYSIS_DELAY_SAMPLES];
    uint32_t input_write_pos;
    uint32_t input_read_pos;
    
    // Audio-Buffer für FFT-Analyse
    float32_t audio_buffer[FFT_SIZE];
    uint32_t audio_buffer_pos;
    
    // Feature-Historie
    FrameFeatures feature_history[FEATURE_HISTORY_SIZE];
    uint32_t feature_write_idx;
    uint32_t feature_count;
    
    // Spektrum-Historie
    float32_t prev_spectrum[FFT_SIZE/2];
    uint8_t prev_spectrum_valid;
    
    // Cluster
    ClusterCenter clusters[MAX_CLUSTERS];
    uint32_t num_clusters;
    uint32_t cluster_update_counter;
    
    // Segmente
    Segment active_segments[50];
    uint32_t active_segment_count;
    uint32_t current_segment_start;
    uint8_t current_cluster_id;
    uint8_t in_segment;
    
    // Globaler Sample-Zähler
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
// GLOBAL VARIABLES
// ============================================================================

CircularBuffer rx_buffer;
CircularBuffer tx_buffer;

// Puffer global definiert um Stack-Overflow zu vermeiden
uint32_t in[BLOCK_SIZE];
uint32_t out[BLOCK_SIZE];
int16_t left_in[BLOCK_SIZE];
int16_t right_in[BLOCK_SIZE];
int16_t left_out[BLOCK_SIZE];
int16_t right_out[BLOCK_SIZE];

// Dummy Buffer für Underruns (Sicherheit)
uint32_t dummy_zeros[BLOCK_SIZE];

// FFT Buffers
float32_t fft_input[FFT_SIZE];
float32_t fft_output[FFT_SIZE * 2];
float32_t spectrum_magnitude[FFT_SIZE/2];

StreamingState state;
arm_rfft_fast_instance_f32 fft_instance;
float32_t hanning_window[FFT_SIZE];

volatile uint32_t processing_overruns = 0;
volatile uint32_t buffer_underruns = 0; // Neu: Zähler für leere Puffer
uint32_t button_debounce_counter = 0;

// ============================================================================
// INITIALIZATION & HELPER
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
// DSP LOGIC (Gekürzt für Übersichtlichkeit, Logik identisch)
// ============================================================================

void extract_features(float32_t* audio_frame, FrameFeatures* features, float32_t* prev_spec, uint32_t sample_pos) {
    float32_t rms;
    arm_rms_f32(audio_frame, FFT_SIZE, &rms);
    features->rms = rms;
    
    if(rms < 0.005f) {
        features->valid = 0;
        // Rest nullen...
        features->centroid = 0; features->rolloff = 0; features->flux = 0; features->zcr = 0;
        features->sample_position = sample_pos;
        return;
    }
    
    features->valid = 1;
    features->sample_position = sample_pos;
    
    // ZCR
    uint32_t zero_crossings = 0;
    for(uint32_t i = 1; i < FFT_SIZE; i++) {
        if((audio_frame[i-1] >= 0.0f && audio_frame[i] < 0.0f) || (audio_frame[i-1] < 0.0f && audio_frame[i] >= 0.0f)) zero_crossings++;
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

// Clustering & Segmentation Code bleibt identisch zum Original...
// (Hier nur Signaturen der Übersichtlichkeit halber, implementation übernehmen!)
float32_t compute_cluster_distance(FrameFeatures* feat, ClusterCenter* cluster);
uint8_t assign_to_cluster(StreamingState* s, FrameFeatures* feat);
void update_cluster_centers(StreamingState* s); // Code aus deiner Vorlage einfügen
void detect_segment_change(StreamingState* s, FrameFeatures* current_feat); // Code aus deiner Vorlage einfügen

// Hier füge ich die Implementierungen wieder ein (stark verkürzt für den Prompt, du nimmst deine Originale):
float32_t compute_cluster_distance(FrameFeatures* feat, ClusterCenter* cluster) {
    float32_t d_cent = (feat->centroid - cluster->centroid) / 10000.0f;
    float32_t d_rms = (feat->rms - cluster->rms) * 5.0f;
    float32_t d_roll = (feat->rolloff - cluster->rolloff) / 10000.0f;
    return WEIGHT_CENTROID * d_cent * d_cent + WEIGHT_RMS * d_rms * d_rms + WEIGHT_ROLLOFF * d_roll * d_roll;
}
uint8_t assign_to_cluster(StreamingState* s, FrameFeatures* feat) {
    if(!feat->valid) return s->current_cluster_id;
    float32_t min_dist = 1e10f; uint8_t best = 0;
    for(uint32_t i = 0; i < s->num_clusters; i++) {
        float32_t dist = compute_cluster_distance(feat, &s->clusters[i]);
        if(dist < min_dist) { min_dist = dist; best = i; }
    }
    return best;
}
// BITTE FÜGE update_cluster_centers UND detect_segment_change HIER EIN (unverändert)
// Ich lasse sie hier im Beispiel weg, um Zeichen zu sparen, da der Fehler woanders liegt.
// Du musst sie aber in deinem Code behalten!
// --- PLATZHALTER FÜR DEINEN ORIGINAL-CODE ZWISCHEN ZEILE 200-350 --- 
// (Ich definiere leere Dummies damit es kompiliert, du nimmst die echten)
void update_cluster_centers(StreamingState* s) { /* Dein Original Code */ }
void detect_segment_change(StreamingState* s, FrameFeatures* f) { /* Dein Original Code */ }


void process_playback_sample(StreamingState* s, int16_t input_sample, int16_t* output_sample, uint32_t global_pos) {
    uint32_t analyzed_pos = (global_pos >= ANALYSIS_DELAY_SAMPLES) ? (global_pos - ANALYSIS_DELAY_SAMPLES) : 0;
    
    uint8_t target_cluster = s->target_cluster;
    uint8_t should_play = 0;
    
    for(uint32_t i = 0; i < s->active_segment_count; i++) {
        Segment* seg = &s->active_segments[i];
        if(analyzed_pos >= seg->start_sample && analyzed_pos < seg->end_sample) {
            if(seg->cluster_id == target_cluster) should_play = 1;
            break;
        }
    }
    
    const float32_t fade_speed = 1.0f / (float32_t)FADE_SAMPLES;
    if(should_play) {
        if(s->fade_gain < 1.0f) {
            s->fade_gain += fade_speed;
            if(s->fade_gain > 1.0f) s->fade_gain = 1.0f;
        }
    } else {
        if(s->fade_gain > 0.0f) {
            s->fade_gain -= fade_speed;
            if(s->fade_gain < 0.0f) s->fade_gain = 0.0f;
        }
    }
    
    float32_t output_f = (float32_t)input_sample * s->fade_gain;
    *output_sample = float_to_int16(output_f * 0.000030518f);
    s->previous_cluster_active = should_play;
}

// ============================================================================
// MAIN
// ============================================================================

int main()
{
    init_platform(115200, USE_SAMPLING_RATE, line_in);
    
    // 1. Initialisierung
    rx_buffer.init();
    tx_buffer.init();
    memset(in, 0, sizeof(in));
    memset(out, 0, sizeof(out));
    memset(dummy_zeros, 0, sizeof(dummy_zeros)); // Dummy Buffer nullen
    
    // 2. FFT Sicherer Init
    arm_status status = arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);
    if (status != ARM_MATH_SUCCESS) {
        // debug_printf("FFT INIT FAILED! Check Size.\n");
        fatal_error(); // Hier ist es okay, weil wir noch nicht gestartet haben
    }
    generate_hanning_window();
    
    // 3. State Init
    init_streaming_state(&state);
    state.target_cluster = 0;

    // ========================================================================
    // CRITICAL FIX: Buffer vorfüllen (Pre-Filling)
    // ========================================================================
    // Wir schreiben 2 volle Blöcke Stille in den Ausgang, bevor wir den Stream starten.
    // Das verhindert, dass der DMA sofort "leer" läuft und fatal_error ruft.
    memset(out, 0, sizeof(out));
    tx_buffer.write(out);
    tx_buffer.write(out);
    
    // debug_printf("Buffers primed. Starting stream...\n");
    platform_start(); // Jetzt erst Interrupts aktivieren
    
    while(true)
    {
        // Blockierendes Lesen vermeiden, aber effizient bleiben
        if(!rx_buffer.read(in)) {
            continue; 
        }
        
        gpio_set(LED_B, HIGH);
        convert_audio_sample_to_2ch(in, left_in, right_in);
        
        // --- Process Block ---
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
                
                // Aufruf der Original-Funktionen (stell sicher, dass sie oben definiert sind)
                detect_segment_change(&state, &current_features);
                
                if(++state.cluster_update_counter >= CLUSTER_UPDATE_RATE) {
                    update_cluster_centers(&state);
                    state.cluster_update_counter = 0;
                }
                
                memmove(state.audio_buffer, &state.audio_buffer[FFT_HOP_SIZE], (FFT_SIZE - FFT_HOP_SIZE) * sizeof(float32_t));
                state.audio_buffer_pos = FFT_SIZE - FFT_HOP_SIZE;
            }
            
            int16_t delayed_sample = ring_read(state.input_ring, ANALYSIS_DELAY_SAMPLES, &state.input_read_pos);
            process_playback_sample(&state, delayed_sample, &left_out[n], state.total_samples_processed);
            right_out[n] = left_out[n];
            state.total_samples_processed++;
        }
        
        // Output
        convert_2ch_to_audio_sample(left_out, right_out, out);
        
        // Write Check
        if(!tx_buffer.write(out)) {
            processing_overruns++;
        }
        
        gpio_set(LED_B, LOW);
        
        // Status print (reduziert, um UART nicht zu fluten)
        #ifdef DEBUG
        static uint32_t loop_cnt = 0;
        if(++loop_cnt > 2000) {
            // debug_printf("Underruns: %lu | Overruns: %lu\n", buffer_underruns, processing_overruns);
            loop_cnt = 0;
        }
        #endif
    }
    
    return 0;
}

// ============================================================================
// CRITICAL FIX: Safe Callbacks
// ============================================================================
// Diese Funktionen werden vom Interrupt aufgerufen. Wir dürfen hier NIEMALS
// fatal_error() aufrufen, wenn der Puffer leer ist, sonst stürzt das System beim Start ab.

uint32_t* get_new_tx_buffer_ptr()
{
    uint32_t* temp = tx_buffer.get_read_ptr();
    if(temp == NULL) {
        // SCHUTZMECHANISMUS: Statt Absturz, gib einen leeren Puffer zurück
        buffer_underruns++;
        return dummy_zeros; 
    }
    return temp;
}

uint32_t* get_new_rx_buffer_ptr()
{
    uint32_t* temp = rx_buffer.get_write_ptr();
    if(temp == NULL) {
        // Wenn kein Platz zum Schreiben ist, gib den Dummy zurück (Daten werden verworfen)
        // Besser als Absturz
        return dummy_zeros; 
    }
    return temp;
}