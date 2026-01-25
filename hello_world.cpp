// Gruppe 3: Hadeed, Patrick, Philipp, Theo, Zihang
// Version: 75 
// Abgabeversion 
//
// Anzahl an Originalsignalen muss angegeben werden. Zeile 25

#include "global.h"
#include "arm_math.h"
#include <math.h>

// Wie viele FFT Blöcke von A in Ausgabe gepuffert werden
#define SIGNAL_ROWS 160

// CFAR Parameter für Start-Erkennung
#define CFAR_WINDOW_SIZE 20
#define CFAR_GUARD_CELLS 2
#define CFAR_THRESHOLD_FACTOR 10.0f

// CFAR Parameter für End-Erkennung
#define END_DETECTION_FRAMES 100            
#define END_ENERGY_THRESHOLD_FACTOR 0.3f

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Konfigurierbare Parameter
uint16_t Amount_of_Signals = 2;

// Allgemeine FFT Parameter
float32_t fs = 32000.0f;
float32_t df = fs / float32_t(BLOCK_SIZE);
float32_t dT = 1.0f/df;

float32_t alpha = 0.3;

// Parameter für feature Gewichtung
uint16_t weightCentroid = 3;
uint16_t weightPower = 2;
uint16_t weightEnergy = 1;
uint16_t weightpeakF = 2;
uint16_t weightFlux = 2;
uint16_t weightbandwidth = 3;
uint16_t weightRolloff = 1;

float32_t score = 0.0f;
float32_t score_threshold = 0.5f;

uint16_t threshCentroid = 1000;
uint16_t threshPower = 10;
uint16_t threshEnergy = 20;
uint16_t threshpeakF = 1700;
uint16_t threshFlux = 1;
uint16_t threshBandwidth = 1000;
uint16_t threshRolloff = 800;

uint16_t flux_counter = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CFAR Variablen
float32_t energy_history[CFAR_WINDOW_SIZE] = {0.0f};
uint32_t energy_history_index = 0;
uint32_t energy_history_count = 0;
bool signal_detected = false;
uint16_t Signal_count = 0;
uint16_t min_Signal_count = 25;

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
int16_t left_out[BLOCK_SIZE];

// FFT buffers and structures
float32_t left_fft[BLOCK_SIZE * 2] = {0.0f};
float32_t left_float[BLOCK_SIZE];

// Komprimierter Ausgangspuffer (16-bit statt 32-bit float)
// Spart 50% RAM: 160 Blöcke × 256 int16 × 2 Bytes = 163 KB statt 327 KB (bei insgesamt 188 KB RAM)
int16_t signalAusgang_compressed[SIGNAL_ROWS][BLOCK_SIZE*2] = {0};
uint32_t write_index = 0;

// Temporärer Dekompression-Buffer
float32_t decompression_buffer[BLOCK_SIZE * 2];

// Feature extraction buffers
float32_t magnitude[BLOCK_SIZE / 2];
float32_t magnitude_past[BLOCK_SIZE / 2] = {0.0f};

// ARM CMSIS DSP FFT instances
arm_rfft_fast_instance_f32 FFT_conf;

// Fensterfunktion (by default Rechteck)
float32_t window[BLOCK_SIZE] = {1.0f};

float32_t fft_input_buffer[BLOCK_SIZE];

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
} AudioFeatures;

float32_t magnitude_sum_past = 0.0f;

AudioFeatures left_features;
AudioFeatures signalA;

// Ausgabe-State
uint32_t playback_index = 0;
bool is_playing = false;

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


// Funktion zur Berechnung der Audio Features
void extract_features(float32_t* mag, float32_t* mag_past, AudioFeatures* features)
{
    const float32_t EPS = 1e-6f;

    // Spectral centroid
    float32_t magnitude_sum = 0.0f;
    float32_t weighted_sum = 0.0f;
    // energy and power
    float32_t power_sum = 0.0f;
    // Peak Frequency index and amplitude
    float32_t peak_mag = 0.0f;
    uint32_t peak_index = 0;
    // Flux
    float32_t flux_sum = 0.0f;
    magnitude_sum_past = magnitude_sum;
    // bandwidth
    bool f_min_found = false;
    float32_t f_min = 0.0f;
    float32_t f_max = 0.0f;


    // feature calculation
    for(uint32_t i = 1; i < BLOCK_SIZE / 2; i++)
    {
        // spectral centroid
        if (mag[i] > 0.01f) {magnitude_sum += mag[i];}
        if(mag[i] > 0.01f) {weighted_sum += (i * df) * mag[i];}

        // peak frequency index + magnitude
        if(mag[i] > peak_mag) {
            peak_mag = mag[i];
            peak_index = i;
        }

        // power sum for energy and power
        power_sum += mag[i] * mag[i];

        // flux
        flux_sum += (mag[i] - mag_past[i]) * (mag[i] - mag_past[i]);

        // bandwidth (everything within 3dB of peak magnitude)
        if(mag[i] > (peak_mag * 0.707f) && !f_min_found) {
            f_min = i * df;
            f_min_found = true;
        }
        if(mag[i] > (peak_mag * 0.707f)){
            f_max = i * df;
        }
        
    }

    features->spectral_rolloff = fs * 0.5f;
    float32_t acc = 0.0f;
    for(uint32_t i = 1; i < BLOCK_SIZE / 2; i++)
    {
        acc += mag[i];
        if(acc >= 0.85f * magnitude_sum) {
            features->spectral_rolloff = (float32_t)i * df;
            break;
        }
    }

    // spectral centroid
    features->spectral_centroid = (magnitude_sum > EPS) ? (weighted_sum / magnitude_sum) : 0.0f;
    // flux
    features->flux = sqrtf(flux_sum) / (magnitude_sum + EPS);
    // bandwith
    features->spectral_bandwidth = f_max-f_min;
    // energy, power, peak frequency index + amplitude
    features->energy = power_sum;
    features->power = 10.0f * log10f(power_sum / (BLOCK_SIZE / 2.0f) + EPS); // in dB
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
    
    // windowfunctions
    arm_hamming_f32(window, BLOCK_SIZE);
    // arm_hanning_f32(window, BLOCK_SIZE);
    
    debug_printf("%s, %s\n", __DATE__, __TIME__);
    IF_DEBUG(debug_printf("fs: %.2f Hz, N: %d\n", fs, BLOCK_SIZE));
    IF_DEBUG(debug_printf("df: %.2f Hz, dt: %.2f ms\n", df, dT * 1000.0));
    IF_DEBUG(debug_printf("Buffer: %d rows = %.2f sec (compressed)\n", SIGNAL_ROWS,(SIGNAL_ROWS) / fs));
    
    platform_start();
    signal_detected = false;
    
    while(true)
    {
        // Step 1: Read block
        while(!rx_buffer.read(in));
        
        gpio_set(LED_B, HIGH);
        gpio_set(TEST_PIN, HIGH);
        
        // Step 2: Split channels
        convert_audio_sample_to_2ch(in, left_in, left_in);

        for(uint32_t n = 0; n < BLOCK_SIZE; n++)
        {
            fft_input_buffer[n] = (float32_t)left_in[n] / 32768.0f;
            left_float[n] = fft_input_buffer[n] * window[n];                        
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
            convert_2ch_to_audio_sample(left_out, left_out, out);
            
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
                debug_printf("Signal Start\n");
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
                debug_printf("Signal Ende\n");
                signal_detected = false;
                reference_energy_set = false;
                low_energy_counter = 0;
                Signal_count = 0;
                write_index = 0;
                
                
                memset(left_out, 0, sizeof(left_out));
                convert_2ch_to_audio_sample(left_out, left_out, out);
                while(!tx_buffer.write(out));
                
                gpio_set(LED_B, LOW);
                gpio_set(TEST_PIN, LOW);
                
                continue;
            }
        }

        // Step 10: Signal A Detection
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

        // Step 11: Signal speichern wenn erkannt (MIT KOMPRESSION)
        if (score > score_threshold && Signal_count == min_Signal_count)
        {
            //IF_DEBUG(debug_printf("Signal_A\n"));
            compress_fft(left_fft, signalAusgang_compressed[write_index], BLOCK_SIZE * 2);

            signalA.spectral_centroid = (1.0f-alpha)*signalA.spectral_centroid + alpha*left_features.spectral_centroid;
            signalA.power = (1.0f-alpha)*signalA.power + alpha*left_features.power;
            signalA.energy = (1.0f-alpha)*signalA.energy + alpha*left_features.energy;
            signalA.peak_frequency = (1.0f-alpha)*signalA.peak_frequency + alpha*left_features.peak_frequency;
            signalA.spectral_bandwidth = (1.0f-alpha)*signalA.spectral_bandwidth + alpha*left_features.spectral_bandwidth;
            signalA.spectral_rolloff = (1.0f-alpha)*signalA.spectral_rolloff + alpha*left_features.spectral_rolloff;
            signalA.peak_magnitude = (1.0f-alpha)*signalA.peak_magnitude + alpha*left_features.peak_magnitude;
            signalA.flux = (1.0f-alpha)*signalA.flux + alpha*left_features.flux;

            write_index++;
            if(write_index >= SIGNAL_ROWS)
            {                
                write_index = 0;
                playback_index = 0;
                is_playing = true;
            }
        }

        // Step 12: Ausgabe (MIT DEKOMPRESSION)
        if(is_playing)
        {
            //debug_printf("Signal Ausgabe\n");
            decompress_fft(signalAusgang_compressed[playback_index], decompression_buffer, BLOCK_SIZE * 2);
            
            // IFFT
            arm_rfft_fast_f32(&FFT_conf, decompression_buffer, left_float, 1);
            
            for(uint32_t i = 0; i < BLOCK_SIZE; i++)
            {
                float32_t sample = left_float[i];
                
                if(sample > 1.0f) sample = 1.0f;
                if(sample < -1.0f) sample = -1.0f;

                int16_t out_sample = (int16_t)(sample * 32767.0f);

                left_out[i] = out_sample;
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
        }

        // Step 13: Merge channels
        convert_2ch_to_audio_sample(left_out, left_out, out);
        
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