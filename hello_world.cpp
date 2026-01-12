/*
 * Author: DSP Gruppe 3
 * Modified: Spectral feature extraction - FIXED VERSION
 * Berechnet: Spectral Centroid, Power, Energy, Frequenzpeak
 * 
 * 4.1 - Bugfixes
 * 
 */
#include "global.h"
#include "arm_math.h"
#include <math.h>

// Wie viele Blöcke A in Ausgabe gepuffert werden
#define SIGNAL_ROWS 80

// CFAR Parameter für Start-Erkennung
#define CFAR_WINDOW_SIZE 20
#define CFAR_GUARD_CELLS 2
#define CFAR_THRESHOLD_FACTOR 10.0f

// CFAR Parameter für End-Erkennung
#define END_DETECTION_FRAMES 200
#define END_ENERGY_THRESHOLD_FACTOR 0.3f

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Konfigurierbare Parameter

// Allgemeine FFT Parameter
float32_t fs = 32000.0f;
float32_t df = fs / float32_t(BLOCK_SIZE);
float32_t dT = 1.0f/df;

// Parameter für feature Gewichtung
uint16_t weightCentroid = 3;
uint16_t weightPower = 2;
uint16_t weightEnergy = 1;
uint16_t weightpeakF = 2;
uint16_t weightFlux = 2;
uint16_t weightbandwidth = 3;
uint16_t weightRolloff = 1;
uint16_t weightFlatness = 0;

float32_t score = 0.0f;
float32_t score_threshold = 0.6f;

uint16_t threshCentroid = 638;
uint16_t threshPower = 4;
uint16_t threshEnergy = 5;
uint16_t threshpeakF = 1733;
uint16_t threshFlux = 1;
uint16_t threshBandwidth = 1000;
uint16_t threshRolloff = 800;
float32_t threshFlatness = 0.3f;  // FIX: war uint16_t, sollte float32_t sein

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CFAR Variablen
float32_t energy_history[CFAR_WINDOW_SIZE] = {0.0f};
uint32_t energy_history_index = 0;
uint32_t energy_history_count = 0;
bool signal_detected = false;
int Signal_count = 0;
int min_Signal_count = 50;

// Variablen für End-Erkennung
uint32_t low_energy_counter = 0;
float32_t signal_energy_reference = 0.0f;
bool reference_energy_set = false;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Circular buffers
CircularBuffer rx_buffer;
CircularBuffer tx_buffer;

// Audio buffers
uint32_t in[BLOCK_SIZE];
uint32_t out[BLOCK_SIZE];
int16_t left_in[BLOCK_SIZE];
int16_t right_in[BLOCK_SIZE];
int16_t left_out[BLOCK_SIZE];
int16_t right_out[BLOCK_SIZE];

// FFT buffers and structures
float32_t left_fft[BLOCK_SIZE * 2] = {0.0f};
float32_t right_fft[BLOCK_SIZE * 2] = {0.0f};
float32_t left_fftPast[BLOCK_SIZE * 2] = {0.0f};  // FIX: Initialisierung hinzugefügt
float32_t right_fftPast[BLOCK_SIZE * 2] = {0.0f};  // FIX: Initialisierung hinzugefügt
float32_t left_float[BLOCK_SIZE];
float32_t right_float[BLOCK_SIZE];

// Ausgangssignal zusammengesetzt
float32_t signalAusgang[SIGNAL_ROWS][BLOCK_SIZE*2] = {0};
uint32_t write_index = 0;

// Feature extraction buffers
float32_t magnitude[BLOCK_SIZE / 2];
float32_t magnitude_past[BLOCK_SIZE / 2] = {0.0f};  // FIX: Initialisierung hinzugefügt

// ARM CMSIS DSP FFT instances
arm_rfft_fast_instance_f32 FFT_conf;

// Fensterfunktion
float32_t window[BLOCK_SIZE] = {1.0f};

// Feature results
typedef struct {
    float32_t spectral_centroid;
    float32_t spectral_bandwidth;
    float32_t spectral_flatness;
    float32_t spectral_rolloff;
    float32_t power;
    float32_t energy;
    float32_t peak_frequency;
    float32_t peak_magnitude;
    float32_t flux;
    float32_t spectral_centroid_past;
    float32_t spectral_bandwidth_past;
    float32_t spectral_flatness_past;
    float32_t spectral_rolloff_past;
    float32_t power_past;
    float32_t energy_past;
    float32_t peak_frequency_past;
    float32_t peak_magnitude_past;
    float32_t flux_past;
} AudioFeatures;

AudioFeatures left_features = {0};   // FIX: Initialisierung hinzugefügt
AudioFeatures right_features = {0};  // FIX: Initialisierung hinzugefügt
AudioFeatures signalA = {0};         // FIX: Initialisierung hinzugefügt

// Ausgabe-State
uint32_t playback_index = 0;
bool is_playing = false;

// Funktion zur Berechnung der Magnitude aus FFT
void calculate_magnitude(float32_t* fft_data, float32_t* mag, uint32_t fft_size)
{
    mag[0] = fabsf(fft_data[0]); // DC component
    
    for(uint32_t i = 1; i < fft_size / 2; i++)
    {
        mag[i] = sqrtf(fft_data[2*i] * fft_data[2*i] + fft_data[2*i+1] * fft_data[2*i+1]);
    }
}

float32_t magnitude_sum_past = 0.0f;

// Funktion zur Berechnung der Audio Features
void extract_features(float32_t* mag, float32_t* mag_past, AudioFeatures* features)
{
    const float32_t EPS = 1e-12f;
    const uint32_t K = BLOCK_SIZE / 2;
    const float32_t invK = 1.0f / (float32_t)(K - 1);

    float32_t weighted_sum = 0.0f;
    float32_t magnitude_sum = 0.0f;
    float32_t power_sum = 0.0f;
    float32_t peak_mag = 0.0f;
    uint32_t peak_index = 0;
    float32_t flux_sum = 0.0f; 
    float32_t sum_log = 0.0f;
    float32_t sum_bw = 0.0f;

    // Merkmale des vorherigen FFT Blocks speichern
    features->spectral_centroid_past = features->spectral_centroid;
    features->power_past = features->power;
    features->energy_past = features->energy;
    features->peak_frequency_past = features->peak_frequency;
    features->peak_magnitude_past = features->peak_magnitude;
    features->flux_past = features->flux;

    // PRE-CALCULATE sum (thresholded)
    for(uint32_t i = 1; i < K; i++)
    {
        if (mag[i] > 0.01f)
            magnitude_sum += mag[i];
    }

    // CALCULATE features
    for(uint32_t i = 1; i < K; i++)    
    {        
        float32_t m = mag[i] + EPS;
        sum_log += logf(m);

        if(mag[i] > 0.01f)
            weighted_sum += (i * df) * mag[i];

        power_sum += mag[i] * mag[i];

        if(mag[i] > peak_mag) {
            peak_mag = mag[i];
            peak_index = i;
        }

        float32_t norm_curr = (magnitude_sum > 1e-6f) ? (mag[i] / magnitude_sum) : 0.0f;
        float32_t norm_past = (magnitude_sum_past > 1e-6f) ? (mag_past[i] / magnitude_sum_past) : 0.0f;
        
        flux_sum += (norm_curr - norm_past) * (norm_curr - norm_past); 
    }

    // Spectral Flatness
    float32_t geo = expf(sum_log * invK);
    float32_t arith = magnitude_sum * invK;
    features->spectral_flatness = geo / (arith + EPS);

    // Spectral Rolloff
    features->spectral_rolloff = fs * 0.5f;
    float32_t acc = 0.0f;
    float32_t thr = 0.85f * magnitude_sum;
    for(uint32_t i = 1; i < K; i++)
    {
        acc += mag[i];
        if(acc >= thr) {
            features->spectral_rolloff = (float32_t)i * df;
            break;
        }
    }

    magnitude_sum_past = magnitude_sum;

    // Spectral Centroid
    features->spectral_centroid = (magnitude_sum > 1e-6f) ? (weighted_sum / magnitude_sum) : 0.0f;
    
    // Spectral Bandwidth
    for(uint32_t i = 1; i < K; i++)
    {
        float32_t diff = (float32_t)i * df - features->spectral_centroid;
        sum_bw += diff * diff * mag[i];
    }
    features->spectral_bandwidth = sqrtf(sum_bw / (magnitude_sum + EPS));
    
    // Energy, Power, Peak
    features->energy = power_sum;
    features->power = 10.0f * log10f(power_sum / (BLOCK_SIZE / 2.0f) + EPS);  // FIX: EPS hinzugefügt
    features->peak_frequency = peak_index * df;
    features->peak_magnitude = peak_mag;
    features->flux = flux_sum;
}

// Normalisierung
double normalize(double value, double min, double max) {
    double n = (value - min) / (max - min);
    
    if (n < 0.0) return 0.0;
    if (n > 1.0) return 1.0;
    
    return n;
}

// CFAR Detektor
bool cfar_detector(float32_t current_energy)
{
    energy_history[energy_history_index] = current_energy;
    energy_history_index = (energy_history_index + 1) % CFAR_WINDOW_SIZE;
    
    if(energy_history_count < CFAR_WINDOW_SIZE)
    {
        energy_history_count++;
    }
    
    if(energy_history_count < CFAR_WINDOW_SIZE)
    {
        return false;
    }
    
    float32_t sum = 0.0f;
    uint32_t count = 0;
    
    for(uint32_t i = 0; i < CFAR_WINDOW_SIZE; i++)
    {
        int32_t distance = (int32_t)energy_history_index - (int32_t)i - 1;
        if(distance < 0) distance += CFAR_WINDOW_SIZE;
        
        if(i >= CFAR_GUARD_CELLS)
        {
            sum += energy_history[distance];
            count++;
        }
    }
    
    float32_t mean_energy = (count > 0) ? (sum / count) : 0.0f;
    float32_t threshold = mean_energy * CFAR_THRESHOLD_FACTOR;
    
    return (current_energy > threshold);
}

// Funktion zur End-Erkennung
bool check_signal_end(float32_t current_energy)
{
    if(!reference_energy_set && signal_detected)
    {
        signal_energy_reference = current_energy;
        reference_energy_set = true;
        low_energy_counter = 0;
        return false;
    }
    
    float32_t end_threshold = signal_energy_reference * END_ENERGY_THRESHOLD_FACTOR;
    
    if(current_energy < end_threshold)
    {
        low_energy_counter++;
        
        if(low_energy_counter >= END_DETECTION_FRAMES)
        {
            IF_DEBUG(debug_printf("SIGNAL ENDE erkannt nach %u Frames\n", low_energy_counter));
            return true;
        }
    }
    else
    {
        low_energy_counter = 0;
        signal_energy_reference = 0.9f * signal_energy_reference + 0.1f * current_energy;
    }
    
    return false;
}

int main()
{
    // Initialize platform
    init_platform(115200, hz32000, line_in);
    
    gpio_set(TEST_PIN, LOW);
    
    // Initialize circular buffers
    rx_buffer.init();
    tx_buffer.init();
    memset(in, 0, sizeof(in));
    memset(out, 0, sizeof(out));
    
    // Initialize FFT instance
    arm_status status = arm_rfft_fast_init_f32(&FFT_conf, BLOCK_SIZE);
    
    // Fensterfunktion
    arm_hamming_f32(window, BLOCK_SIZE);
    
    IF_DEBUG(debug_printf("fs: %.2f Hz, N: %d\n", fs, BLOCK_SIZE));
    IF_DEBUG(debug_printf("df: %.2f Hz, dt: %.2f ms\n", df, dT * 1000.0));
    
    platform_start();
    signal_detected = false;
    
    while(true)
    {
        // Step 1: Read block
        while(!rx_buffer.read(in));
        
        gpio_set(LED_B, HIGH);
        gpio_set(TEST_PIN, HIGH);
        
        // Step 2: Split channels
        convert_audio_sample_to_2ch(in, left_in, right_in);
        
        // Step 3: Convert to float with windowing
        for(uint32_t n = 0; n < BLOCK_SIZE; n++)
        {
            left_float[n] = ((float32_t)left_in[n] / 32768.0f) * window[n];
        }

        // Step 4: FFT
        arm_rfft_fast_f32(&FFT_conf, left_float, left_fft, 0);
        
        // Step 5: Calculate magnitude
        calculate_magnitude(left_fft, magnitude, BLOCK_SIZE);

        // Step 6: Extract features
        extract_features(magnitude, magnitude_past, &left_features);
        memcpy(magnitude_past, magnitude, sizeof(magnitude));

        // Step 7: CFAR Signal Detection
        if(!signal_detected)
        {
            signal_detected = cfar_detector(left_features.energy);
            
            memset(left_out, 0, sizeof(left_out));
            memset(right_out, 0, sizeof(right_out));
            convert_2ch_to_audio_sample(left_out, right_out, out);
            
            while(!tx_buffer.write(out));
            
            gpio_set(LED_B, LOW);
            gpio_set(TEST_PIN, LOW);
            
            continue;
        }
        
        // Step 8: Erstmaliges Einlesen von Signal A
        if(signal_detected && Signal_count < min_Signal_count)
        {
            if(Signal_count == (min_Signal_count - 1))
            {
                IF_DEBUG(debug_printf("Signal erkannt. Erste Features eingeschrieben\n"));
                signalA.energy = left_features.energy;
                signalA.spectral_bandwidth = left_features.spectral_bandwidth;
                signalA.spectral_flatness = left_features.spectral_flatness;
                signalA.spectral_rolloff = left_features.spectral_rolloff;
                signalA.power = left_features.power;
                signalA.spectral_centroid = left_features.spectral_centroid;
                signalA.peak_frequency = left_features.peak_frequency;
                signalA.peak_magnitude = left_features.peak_magnitude;
                signalA.flux = left_features.flux;
                
                IF_DEBUG(debug_printf("BW: %.2f Hz\n", signalA.spectral_bandwidth));
                IF_DEBUG(debug_printf("RO: %.2f Hz\n", signalA.spectral_rolloff));
                IF_DEBUG(debug_printf("FL: %.2f\n", signalA.spectral_flatness));
                IF_DEBUG(debug_printf("\n"));
            }
            Signal_count++;
        }
        
        // Step 9: End-Erkennung
        if(signal_detected && Signal_count >= min_Signal_count)
        {
            if(check_signal_end(left_features.energy))
            {
                signal_detected = false;
                reference_energy_set = false;
                low_energy_counter = 0;
                Signal_count = 0;
                write_index = 0;
                
                IF_DEBUG(debug_printf("=== SIGNAL KOMPLETT BEENDET - System Reset ===\n"));
                
                memset(left_out, 0, sizeof(left_out));
                memset(right_out, 0, sizeof(right_out));
                convert_2ch_to_audio_sample(left_out, right_out, out);
                while(!tx_buffer.write(out));
                
                gpio_set(LED_B, LOW);
                gpio_set(TEST_PIN, LOW);
                
                continue;
            }
        }

        // Step 10: Signal A Detection
        score = 0.0f;

        // FIX: Gewichtungen korrigiert - vorher war weightpeakF für alle verwendet
        if(fabs(left_features.spectral_centroid - signalA.spectral_centroid) < threshCentroid) {
            score += weightCentroid * (1.0f - normalize(fabs(left_features.spectral_centroid - signalA.spectral_centroid), 0, threshCentroid));
        }

        if(fabs(left_features.power - signalA.power) < threshPower) {
            score += weightPower * (1.0f - normalize(fabs(left_features.power - signalA.power), 0, threshPower)); 
        }

        if(fabs(left_features.energy - signalA.energy) < threshEnergy) {
            score += weightEnergy * (1.0f - normalize(fabs(left_features.energy - signalA.energy), 0, threshEnergy));
        }

        if(fabs(left_features.peak_frequency - signalA.peak_frequency) < threshpeakF) {
            score += weightpeakF * (1.0f - normalize(fabs(left_features.peak_frequency - signalA.peak_frequency), 0, threshpeakF));
        }

        if(fabs(left_features.spectral_bandwidth - signalA.spectral_bandwidth) < threshBandwidth) {
            score += weightbandwidth * (1.0f - normalize(fabs(left_features.spectral_bandwidth - signalA.spectral_bandwidth), 0, threshBandwidth));
        }

        if(fabs(left_features.spectral_rolloff - signalA.spectral_rolloff) < threshRolloff) {
            score += weightRolloff * (1.0f - normalize(fabs(left_features.spectral_rolloff - signalA.spectral_rolloff), 0, threshRolloff));
        }

        if(fabs(left_features.spectral_flatness - signalA.spectral_flatness) < threshFlatness) {
            score += weightFlatness * (1.0f - normalize(fabs(left_features.spectral_flatness - signalA.spectral_flatness), 0, threshFlatness));
        }

        if(fabs(left_features.flux) < threshFlux) {
            score += weightFlux * (1.0f - normalize(fabs(left_features.flux), 0, threshFlux));
        }

        score = score / (weightCentroid + weightPower + weightEnergy + weightpeakF + weightFlux + weightbandwidth + weightFlatness + weightRolloff);

        // Step 11: Debug Output
        static uint32_t frame_count = 0;
        frame_count++;
        if(frame_count % 20 == 0)
        {
            IF_DEBUG(debug_printf("SC: %.2f Hz\n", left_features.spectral_centroid));
            IF_DEBUG(debug_printf("P: %.2f dB\n", left_features.power));
            IF_DEBUG(debug_printf("PF: %.2f Hz (Mag: %.2f)\n", left_features.peak_frequency, left_features.peak_magnitude));
            IF_DEBUG(debug_printf("F: %.2f\n", left_features.flux));
            IF_DEBUG(debug_printf("RO: %.2f Hz\n", left_features.spectral_rolloff));
            IF_DEBUG(debug_printf("BW: %.2f Hz\n", left_features.spectral_bandwidth));
            IF_DEBUG(debug_printf("Score: %.2f\n", score));
            IF_DEBUG(debug_printf("\n"));
        }

        // Step 12: Signal speichern wenn erkannt
        // FIX: "and" zu "&&" geändert
        if (score > score_threshold && Signal_count == min_Signal_count)
        {
            memcpy(signalAusgang[write_index], left_fft, BLOCK_SIZE * 2 * sizeof(float32_t));

            // Features aktualisieren (gleitender Durchschnitt)
            signalA.energy = (left_features.energy + signalA.energy) / 2.0f;
            signalA.power = (left_features.power + signalA.power) / 2.0f;
            signalA.spectral_bandwidth = (left_features.spectral_bandwidth + signalA.spectral_bandwidth) / 2.0f;
            signalA.spectral_flatness = (left_features.spectral_flatness + signalA.spectral_flatness) / 2.0f;
            signalA.spectral_rolloff = (left_features.spectral_rolloff + signalA.spectral_rolloff) / 2.0f;
            signalA.spectral_centroid = (left_features.spectral_centroid + signalA.spectral_centroid) / 2.0f;
            signalA.peak_frequency = (left_features.peak_frequency + signalA.peak_frequency) / 2.0f;
            signalA.peak_magnitude = (left_features.peak_magnitude + signalA.peak_magnitude) / 2.0f;
            signalA.flux = (left_features.flux + signalA.flux) / 2.0f;

            write_index++;
            if(write_index >= SIGNAL_ROWS)
            {                
                IF_DEBUG(debug_printf("SignalA Komplett - starte Wiedergabe\n"));
                write_index = 0;
                playback_index = 0;
                is_playing = true;
            }
        }

        // Step 13: Ausgabe
        if(is_playing)
        {
            arm_rfft_fast_f32(&FFT_conf, signalAusgang[playback_index], left_float, 1);
            
            for(uint32_t i = 0; i < BLOCK_SIZE; i++)
            {
                float32_t sample = left_float[i];
                
                if(sample > 1.0f) sample = 1.0f;
                if(sample < -1.0f) sample = -1.0f;

                int16_t out_sample = (int16_t)(sample * 32767.0f);

                left_out[i] = out_sample;
                right_out[i] = out_sample;
            }
            
            playback_index++;
            if(playback_index >= SIGNAL_ROWS)
            {
                IF_DEBUG(debug_printf("Wiedergabe beendet\n"));
                playback_index = 0;
                is_playing = false;
            }
        }
        else
        {
            memset(left_out, 0, sizeof(left_out));
            memset(right_out, 0, sizeof(right_out));
        }

        // Step 14: Merge channels
        convert_2ch_to_audio_sample(left_out, right_out, out);
        
        // Step 15: Write output
        while(!tx_buffer.write(out));
        
        gpio_set(LED_B, LOW);
        gpio_set(TEST_PIN, LOW);
    }
    
    fatal_error();
    return 0;
}

// DMA callback functions
uint32_t* get_new_tx_buffer_ptr()
{
    uint32_t* temp = tx_buffer.get_read_ptr();
    if(temp == nullptr)
    {
        fatal_error();
    }
    return temp;
}

uint32_t* get_new_rx_buffer_ptr()
{
    uint32_t* temp = rx_buffer.get_write_ptr();
    if(temp == nullptr)
    {
        fatal_error();
    }
    return temp;
}