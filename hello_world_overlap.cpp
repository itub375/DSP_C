//
// Overlap korrektur
//
//

#include "global.h"
#include "arm_math.h"
#include <math.h>

// *** GEÄNDERT: Doppelte Buffer-Größe durch Kompression ***
#define SIGNAL_ROWS 80  // 240 * 4 ms  = 

// CFAR Parameter für Start-Erkennung
#define CFAR_WINDOW_SIZE 20
#define CFAR_GUARD_CELLS 2
#define CFAR_THRESHOLD_FACTOR 10.0f

// End-Erkennung
#define END_DETECTION_FRAMES 100
#define END_ENERGY_THRESHOLD_FACTOR 0.3f

// Overlap-Parameter
#define HOP_SIZE (BLOCK_SIZE / 2)  // 50% Overlap
#define OVERLAP_SIZE (BLOCK_SIZE - HOP_SIZE)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Konfigurierbare Parameter
uint16_t Amount_of_Signals = 2;

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

float32_t score = 0.0f;
float32_t score_overlap = 0.0f;
float32_t score_threshold = 0.5f;

uint16_t threshCentroid = 1000;
uint16_t threshPower = 10;
uint16_t threshEnergy = 20;
uint16_t threshpeakF = 1733;
uint16_t threshFlux = 1;
uint16_t threshBandwidth = 1000;
uint16_t threshRolloff = 800;

uint16_t flux_counter = 0;
uint16_t flux_counter_overlap = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CFAR Variablen
float32_t energy_history[CFAR_WINDOW_SIZE] = {0.0f};
uint32_t energy_history_index = 0;
uint32_t energy_history_count = 0;
bool signal_detected = false;
int Signal_count = 0;
int min_Signal_count = 25;

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

// Overlap-Buffer für kontinuierliche Verarbeitung

float32_t overlap_buffer[BLOCK_SIZE] = {0.0f};
float32_t previous_buffer[BLOCK_SIZE] = {0.0f};
bool overlap_initialized = false;

// FFT buffers and structures
float32_t left_fft[BLOCK_SIZE * 2] = {0.0f};
float32_t over_fft[BLOCK_SIZE * 2] = {0.0f};
float32_t left_first_fft[BLOCK_SIZE] = {0.0f};
float32_t over_first_fft[BLOCK_SIZE] = {0.0f};
float32_t right_fft[BLOCK_SIZE * 2] = {0.0f};
float32_t left_fftPast[BLOCK_SIZE * 2] = {0.0f};
float32_t right_fftPast[BLOCK_SIZE * 2] = {0.0f};
float32_t left_float[BLOCK_SIZE];
float32_t overlap_float[BLOCK_SIZE];
float32_t right_float[BLOCK_SIZE];

bool overflow_speichern = false;
//float32_t prev_saved_left_fft[BLOCK_SIZE] = {0.0f};
float32_t playback_overlap_buffer[HOP_SIZE] = {0.0f};
bool playback_first = true;
bool first_iteration = true;

// Temporärer Buffer für FFT-Input
float32_t fft_input_buffer[BLOCK_SIZE];

// *** GEÄNDERT: Komprimierter Ausgangspuffer (16-bit statt 32-bit float) ***
// Spart 50% RAM: 160 Blöcke × 2048 int16 × 2 Bytes = 655 KB statt 1.31 MB
int16_t signalAusgang_compressed[SIGNAL_ROWS][BLOCK_SIZE*2] = {0};
uint32_t write_index = 0;

// *** NEU: Temporärer Dekompression-Buffer ***
float32_t decompression_buffer[BLOCK_SIZE * 2];

// Feature extraction buffers
float32_t magnitude[BLOCK_SIZE / 2];
float32_t magnitude_past[BLOCK_SIZE / 2] = {0.0f};

// Overlap Feature extraction buffers
float32_t magnitude_overlap[BLOCK_SIZE / 2];
float32_t magnitude_overlap_past[BLOCK_SIZE / 2] = {0.0f};

// ARM CMSIS DSP FFT instances
arm_rfft_fast_instance_f32 FFT_conf;

// Fensterfunktion
float32_t window[BLOCK_SIZE] = {1.0f};

// Feature results
typedef struct {
    float32_t spectral_centroid;
    float32_t spectral_bandwidth;
    float32_t spectral_rolloff;
    float32_t power;
    float32_t energy;
    float32_t peak_frequency;
    float32_t peak_magnitude;
    float32_t flux;
    float32_t spectral_centroid_past;
    float32_t spectral_bandwidth_past;
    float32_t spectral_rolloff_past;
    float32_t power_past;
    float32_t energy_past;
    float32_t peak_frequency_past;
    float32_t peak_magnitude_past;
    float32_t flux_past;
} AudioFeatures;

AudioFeatures left_features;
AudioFeatures overlap_features;
AudioFeatures right_features;
AudioFeatures signalA;

// Ausgabe-State
uint32_t playback_index = 0;
bool is_playing = false;

// *** NEU: Kompression/Dekompression Funktionen ***

// Komprimiert FFT-Daten von float32 zu int16
void compress_fft(float32_t* fft_float, int16_t* fft_int16, uint32_t length)
{
    // Skalierungsfaktor für bessere Auflösung
    // Typische FFT-Werte liegen zwischen -1.0 und +1.0 nach Normalisierung
    const float32_t SCALE = 16384.0f;  // Lässt Headroom für Peaks
    
    for(uint32_t i = 0; i < length; i++)
    {
        float32_t val = fft_float[i] * SCALE;
        
        // Clipping
        if(val > 32767.0f) val = 32767.0f;
        if(val < -32768.0f) val = -32768.0f;
        
        fft_int16[i] = (int16_t)val;
    }
}

// Dekomprimiert FFT-Daten von int16 zu float32
void decompress_fft(int16_t* fft_int16, float32_t* fft_float, uint32_t length)
{
    const float32_t INV_SCALE = 1.0f / 16384.0f;
    
    for(uint32_t i = 0; i < length; i++)
    {
        fft_float[i] = (float32_t)fft_int16[i] * INV_SCALE;
    }
}

// Funktion zur Berechnung der Magnitude aus FFT
void calculate_magnitude(float32_t* fft_data, float32_t* mag, uint32_t fft_size)
{
    mag[0] = fabsf(fft_data[0]);
    
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

    float32_t magnitude_sum = 0.0f;
    float32_t weighted_sum = 0.0f;
    float32_t power_sum = 0.0f;
    float32_t peak_mag = 0.0f;
    uint32_t peak_index = 0;
    float32_t flux_sum = 0.0f;
    float32_t sum_log = 0.0f;
    float32_t invK = 1.0f / (float32_t)(K - 1);
    float32_t sum_bw = 0.0f;
    bool f_min_found = false;
    float32_t f_min = 0.0f;
    float32_t f_max = 0.0f;

    features->spectral_centroid_past = features->spectral_centroid;
    features->power_past = features->power;
    features->energy_past = features->energy;
    features->peak_frequency_past = features->peak_frequency;
    features->peak_magnitude_past = features->peak_magnitude;
    features->flux_past = features->flux;
    features->spectral_bandwidth_past = features->spectral_bandwidth;
    features->spectral_rolloff_past = features->spectral_rolloff;

    magnitude_sum_past = magnitude_sum;

    for(uint32_t i = 1; i < K; i++)
    {
        if (mag[i] > 0.01f) {magnitude_sum += mag[i];}
        if(mag[i] > 0.01f) {weighted_sum += (i * df) * mag[i];}

        if(mag[i] > peak_mag) {
            peak_mag = mag[i];
            peak_index = i;
        }

        power_sum += mag[i] * mag[i];

        if(fabs(mag[i] - mag_past[i]) > 0.0f) { 
            flux_sum += (mag[i] - mag_past[i]) * (mag[i] - mag_past[i]);
        }

        if(mag[i]> (peak_mag * (1.0f/sqrtf(2.0f))) && !f_min_found) {
            f_min = i * df;
            f_min_found = true;
        }
        if(mag[i] > (peak_mag * (1.0f/sqrtf(2.0f)))){
            f_max = i * df;
        }
        
        sum_log += logf(mag[i] + EPS);
    }

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

    features->spectral_centroid = (magnitude_sum > 1e-6f) ? (weighted_sum / magnitude_sum) : 0.0f;
    features->flux = sqrtf(flux_sum) / (magnitude_sum + EPS);

    features->spectral_bandwidth = f_max-f_min;
    features->energy = power_sum;
    features->power = 10.0f * log10f(power_sum / (BLOCK_SIZE / 2.0f) + EPS);
    features->peak_frequency = peak_index * df;
    features->peak_magnitude = peak_mag;
}

double normalize(double value, double min, double max) {
    double n = (value - min) / (max - min);
    
    if (n < 0.0) return 0.0;
    if (n > 1.0) return 1.0;
    
    return n;
}

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

bool check_signal_end(float32_t current_energy, float32_t current_centroid)
{
    if(!reference_energy_set && signal_detected)
    {
        signal_energy_reference = current_energy;
        reference_energy_set = true;
        low_energy_counter = 0;
        return false;
    }
    
    float32_t end_threshold = signal_energy_reference * END_ENERGY_THRESHOLD_FACTOR;
    
    if(current_energy < end_threshold || current_centroid <1)
    {
        low_energy_counter++;
        
        if(low_energy_counter >= END_DETECTION_FRAMES)
        {
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
    init_platform(115200, hz32000, line_in);
    
    gpio_set(TEST_PIN, LOW);
    
    rx_buffer.init();
    tx_buffer.init();
    memset(in, 0, sizeof(in));
    memset(out, 0, sizeof(out));
    
    arm_status status = arm_rfft_fast_init_f32(&FFT_conf, BLOCK_SIZE);
    
    arm_hamming_f32(window, BLOCK_SIZE);
    
    debug_printf("%s, %s\n", __DATE__, __TIME__);
    IF_DEBUG(debug_printf("fs: %.2f Hz, N: %d\n", fs, BLOCK_SIZE));
    IF_DEBUG(debug_printf("df: %.2f Hz, dt: %.2f ms\n", df, dT * 1000.0));
    IF_DEBUG(debug_printf("HOP: %d (50%% overlap)\n", HOP_SIZE));
    IF_DEBUG(debug_printf("Buffer: %d rows = %.2f sec (compressed)\n", SIGNAL_ROWS,(SIGNAL_ROWS * HOP_SIZE) / fs));
    
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


        
        // Step 3: Overlap-Verarbeitung
    
        for(uint32_t n = 0; n < HOP_SIZE; n++)
        {
            overlap_buffer[n] = previous_buffer[HOP_SIZE+ n];                       //vorherhige zweite hälfte in erstehälfte overlapbuffer
            overlap_buffer[OVERLAP_SIZE + n] = (float32_t)left_in[n] / 32768.0f;    //neue erste hälfte in zweite hälfte overlapbuffer
            
        }
                
        for(uint32_t n = 0; n < BLOCK_SIZE; n++)
        {
            previous_buffer[n] = (float32_t)left_in[n] / 32768.0f;       //neue zweite hälfte in previous buffer
            fft_input_buffer[n] = (float32_t)left_in[n] / 32768.0f;                 // neues in input buffer
            
            overlap_float[n] = overlap_buffer[n] * window[n];                       // Fensterung input buffer
            left_float[n] = fft_input_buffer[n] * window[n];                        // Fensterung Overlap
        }





        // Step 4: FFT
        arm_rfft_fast_f32(&FFT_conf, left_float, left_fft, 0);
        arm_rfft_fast_f32(&FFT_conf, overlap_float, over_fft, 0);
        
        // Step 5: Calculate magnitude
        calculate_magnitude(left_fft, magnitude, BLOCK_SIZE);
        calculate_magnitude(over_fft, magnitude_overlap, BLOCK_SIZE);

        // Step 6: Extract features
        extract_features(magnitude, magnitude_past, &left_features);
        memcpy(magnitude_past, magnitude, sizeof(magnitude));
        
        extract_features(magnitude_overlap, magnitude_overlap_past, &overlap_features);
        memcpy(magnitude_overlap_past, magnitude_overlap, sizeof(magnitude_overlap));




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
                IF_DEBUG(debug_printf("Signal Erkannt\n"));
                signalA.energy = left_features.energy;
                signalA.spectral_bandwidth = left_features.spectral_bandwidth;
                signalA.spectral_rolloff = left_features.spectral_rolloff;
                signalA.power = left_features.power;
                signalA.spectral_centroid = left_features.spectral_centroid;
                signalA.peak_frequency = left_features.peak_frequency;
                signalA.peak_magnitude = left_features.peak_magnitude;
                signalA.flux = left_features.flux;
            }
            Signal_count++;
        }
        
        // Step 9: End-Erkennung
        if(signal_detected && Signal_count >= min_Signal_count)
        {
            if(check_signal_end(left_features.energy, left_features.spectral_centroid))
            {
                IF_DEBUG(debug_printf("Signal Stopp\n"));
                signal_detected = false;
                reference_energy_set = false;
                low_energy_counter = 0;
                Signal_count = 0;
                write_index = 0;
                
                memset(overlap_buffer, 0, sizeof(overlap_buffer));
                
                memset(left_out, 0, sizeof(left_out));
                memset(right_out, 0, sizeof(right_out));
                convert_2ch_to_audio_sample(left_out, right_out, out);
                while(!tx_buffer.write(out));
                
                gpio_set(LED_B, LOW);
                gpio_set(TEST_PIN, LOW);
                
                continue;
            }
        }




        // Step 10: Signal A overlap Detection
        score_overlap = 0.0f;

        if(fabs(overlap_features.spectral_centroid - signalA.spectral_centroid) < threshCentroid) {
            score_overlap += weightCentroid * (1.0f - normalize(fabs(overlap_features.spectral_centroid - signalA.spectral_centroid), 0, threshCentroid));
        }

        if(fabs(overlap_features.power - signalA.power) < threshPower) {
            score_overlap += weightPower * (1.0f - normalize(fabs(overlap_features.power - signalA.power), 0, threshPower)); 
        }

        if(fabs(overlap_features.energy - signalA.energy) < threshEnergy) {
            score_overlap += weightEnergy * (1.0f - normalize(fabs(overlap_features.energy - signalA.energy), 0, threshEnergy));
        }

        if(fabs(overlap_features.peak_frequency - signalA.peak_frequency) < threshpeakF) {
            score_overlap += weightpeakF * (1.0f - normalize(fabs(overlap_features.peak_frequency - signalA.peak_frequency), 0, threshpeakF));
        }

        if(fabs(overlap_features.spectral_bandwidth - signalA.spectral_bandwidth) < threshBandwidth) {
            score_overlap += weightbandwidth * (1.0f - normalize(fabs(overlap_features.spectral_bandwidth - signalA.spectral_bandwidth), 0, threshBandwidth));
        }

        if(fabs(overlap_features.spectral_rolloff - signalA.spectral_rolloff) < threshRolloff) {
            score_overlap += weightRolloff * (1.0f - normalize(fabs(overlap_features.spectral_rolloff - signalA.spectral_rolloff), 0, threshRolloff));
        }
        
        if(fabs(overlap_features.flux) > threshFlux) {
            flux_counter_overlap = (flux_counter_overlap + 1)%(Amount_of_Signals);
        }

        if(flux_counter_overlap == 0) {
            score_overlap += weightFlux * (1.0f - normalize(fabs(overlap_features.flux), 0, threshFlux));
        }

        score_overlap = score_overlap / (weightCentroid + weightPower + weightEnergy + weightpeakF + weightFlux + weightbandwidth + weightRolloff);



        // Step 11: Signal speichern wenn erkannt (MIT KOMPRESSION)
        if (score_overlap > score_threshold && Signal_count == min_Signal_count)
        {
            
        IF_DEBUG(debug_printf("Signal_A_overlap\n"));
            // *** GEÄNDERT: FFT-Daten komprimieren vor dem Speichern ***
            compress_fft(over_fft, signalAusgang_compressed[write_index], BLOCK_SIZE*2 );

            signalA.energy = (overlap_features.energy + signalA.energy) / 2.0f;
            signalA.power = (overlap_features.power + signalA.power) / 2.0f;
            signalA.spectral_bandwidth = (overlap_features.spectral_bandwidth + signalA.spectral_bandwidth) / 2.0f;
            signalA.spectral_rolloff = (overlap_features.spectral_rolloff + signalA.spectral_rolloff) / 2.0f;
            signalA.spectral_centroid = (overlap_features.spectral_centroid + signalA.spectral_centroid) / 2.0f;
            signalA.peak_frequency = (overlap_features.peak_frequency + signalA.peak_frequency) / 2.0f;
            signalA.peak_magnitude = (overlap_features.peak_magnitude + signalA.peak_magnitude) / 2.0f;
            signalA.flux = (overlap_features.flux + signalA.flux) / 2.0f;

            write_index++;
                        if(write_index >= SIGNAL_ROWS)
            {
                write_index = 0;
                playback_index = 0;
                is_playing = true;

            }
        }








        // Step 12: Signal A Detection
        score = 0.0f;

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
        
        if(fabs(left_features.flux) > threshFlux) {
            flux_counter = (flux_counter + 1)%(Amount_of_Signals);
        }

        if(flux_counter == 0) {
            score += weightFlux * (1.0f - normalize(fabs(left_features.flux), 0, threshFlux));
        }

        score = score / (weightCentroid + weightPower + weightEnergy + weightpeakF + weightFlux + weightbandwidth + weightRolloff);


        // Step 13: Signal speichern wenn erkannt (MIT KOMPRESSION)
        if (score > score_threshold && Signal_count == min_Signal_count)
        {
            
        IF_DEBUG(debug_printf("Signal_A\n"));
            signalA.energy = (left_features.energy + signalA.energy) / 2.0f;
            signalA.power = (left_features.power + signalA.power) / 2.0f;
            signalA.spectral_bandwidth = (left_features.spectral_bandwidth + signalA.spectral_bandwidth) / 2.0f;
            signalA.spectral_rolloff = (left_features.spectral_rolloff + signalA.spectral_rolloff) / 2.0f;
            signalA.spectral_centroid = (left_features.spectral_centroid + signalA.spectral_centroid) / 2.0f;
            signalA.peak_frequency = (left_features.peak_frequency + signalA.peak_frequency) / 2.0f;
            signalA.peak_magnitude = (left_features.peak_magnitude + signalA.peak_magnitude) / 2.0f;
            signalA.flux = (left_features.flux + signalA.flux) / 2.0f;

            write_index++;
            if(write_index >= SIGNAL_ROWS && is_playing == false)
            {
                write_index = 0;
                playback_index = 0;
                is_playing = true;

            }else{

            // *** GEÄNDERT: FFT-Daten komprimieren vor dem Speichern ***
            compress_fft(left_fft, signalAusgang_compressed[write_index], BLOCK_SIZE *2);

            }
        }





        // Step 12: Ausgabe (MIT DEKOMPRESSION)
        if(is_playing)
        {
            // *** GEÄNDERT: FFT-Daten dekomprimieren vor IFFT ***
            decompress_fft(signalAusgang_compressed[playback_index], decompression_buffer, BLOCK_SIZE*2);
            
            arm_rfft_fast_f32(&FFT_conf, decompression_buffer, left_float, 1);
            
            for(uint32_t i = 0; i < HOP_SIZE; i++)
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
                playback_index = 0;
                is_playing = false;
            }
        }
        else
        {
            memset(left_out, 0, sizeof(left_out));
            memset(right_out, 0, sizeof(right_out));
        }

        // Step 13: Merge channels
        convert_2ch_to_audio_sample(left_out, right_out, out);
        
        // Step 14: Write output
        while(!tx_buffer.write(out));
        
        gpio_set(LED_B, LOW);
        gpio_set(TEST_PIN, LOW);
    }
    
    fatal_error();
    return 0;
}

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