/*
 * Author: Jan Eberhardt
 * Modified: Spectral feature extraction
 * Berechnet: Spectral Centroid, Power, Energy, Frequenzpeak
 */
#include "global.h"
#include "arm_math.h"
#include <math.h>

// typedef enum
// {
//     hz8000  = 0x0C,  // 8kHz from 12.288MHz MCLK
//     hz32000 = 0x18,  // 32kHz from 12.288MHz MCLK
// 	   hz48000 = 0x00,  // 48kHz from 12.288MHz MCLK
//     hz96000 = 0x1C,  // 96kHz from 12.288MHz MCLK
// } sampling_rate;
float fs = 32000.0;


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
float32_t left_float[BLOCK_SIZE];
float32_t right_float[BLOCK_SIZE];
float32_t left_fft[BLOCK_SIZE * 2];   // Complex output (real + imag)
float32_t right_fft[BLOCK_SIZE * 2];  // Complex output (real + imag)

// Feature extraction buffers
float32_t magnitude[BLOCK_SIZE / 2];  // Magnitude spectrum
// Flux buffers
float32_t left_fft_previous[BLOCK_SIZE * 2];   // Komplette FFT Daten
float32_t right_fft_previous[BLOCK_SIZE * 2];  // Falls beide Kanäle
bool first_flux_block = true;

// ARM CMSIS DSP FFT instances
arm_rfft_fast_instance_f32 fft_instance;

float32_t hamming_window[BLOCK_SIZE];

// Feature results
typedef struct {
    float32_t spectral_centroid;
    float32_t power;
    float32_t energy;
    float32_t peak_frequency;
    float32_t peak_magnitude;
    float32_t spectral_flux;
} AudioFeatures;

AudioFeatures left_features;
AudioFeatures right_features;

// Funktion zur Berechnung der Magnitude aus FFT
void calculate_magnitude(float32_t* fft_data, float32_t* mag, uint32_t fft_size)
{
    mag[0] = fabsf(fft_data[0]);//dc 
    // FFT output format: [real0, imag0, real1, imag1, ..., realN/2, imagN/2]
    // Berechne Magnitude für jede Frequenz bin
    for(uint32_t i = 1; i < fft_size / 2; i++)
    {
        float32_t real = fft_data[i * 2];
        float32_t imag = fft_data[i * 2 + 1];
        mag[i] = sqrtf(real * real + imag * imag);
    }
   
   // mag[1]=0.f;
}

// Funktion zur Berechnung der Audio Features
void extract_features(float32_t* mag, AudioFeatures* features)
{
    float32_t weighted_sum = 0.0f;
    float32_t magnitude_sum = 0.0f;
    float32_t power_sum = 0.0f;
    float32_t energy = 0.0f;
    float32_t peak_mag = 0.0f;
    uint32_t peak_index = 0;
    
    float32_t freq_resolution = fs / (BLOCK_SIZE);  // Hz pro bin
    // Threshold to ignore noise in Centroid calculation
    const float32_t silence_threshold = 0.01f;
    for(uint32_t i = 1; i < BLOCK_SIZE/2; i++)
    {
        float32_t freq = i * freq_resolution;
        float32_t mag_val = mag[i];
        if(mag_val > silence_threshold) {
            weighted_sum += (freq) * mag_val;
            magnitude_sum += mag_val;
        }
        // Spectral Centroid: gewichteter Durchschnitt der Frequenzen
        //weighted_sum += freq * mag_val;
        //magnitude_sum += mag_val;
        
        // Power: Summe der quadrierten Magnituden
        power_sum += mag_val * mag_val;
        
        // Peak Frequency: Finde Maximum
        if(mag_val > peak_mag)
        {
            peak_mag = mag_val;
            peak_index = i;
        }
    }
    
    // Spectral Centroid in Hz
    features->spectral_centroid = (magnitude_sum > 1e-6f) ? (weighted_sum / magnitude_sum) : 0.0f;
    
    // Power (mittlere quadratische Magnitude)
    features->power = power_sum / (float32_t)(BLOCK_SIZE / 2);
    
    // Energy (Gesamtenergie)
    features->energy = power_sum;
    
    // Peak Frequency in Hz
    features->peak_frequency = peak_index * freq_resolution;
    features->peak_magnitude = peak_mag;
}

// Funktion zur Berechnung des Spectral Flux
void calculate_fft_flux_optimized(float32_t* fft_current, AudioFeatures* features)
{
    float32_t flux = 0.0f;
    
    if(first_flux_block)
    {
        flux = 0.0f;
        first_flux_block = false;
        IF_DEBUG(debug_printf("FFT Flux initialized\n"));
    }
    else
    {
        // Direkte Berechnung aus komplexen FFT-Daten
        // Format: [real0, imag0, real1, imag1, ...]
        
        for(uint32_t i = 2; i < BLOCK_SIZE; i += 2)  // Start bei 2 (skip DC), step 2
        {
            // Current magnitude (ohne sqrt für Performance)
            float32_t mag_curr_sq = fft_current[i] * fft_current[i] + 
                                    fft_current[i+1] * fft_current[i+1];
            
            // Previous magnitude
            float32_t mag_prev_sq = left_fft_previous[i] * left_fft_previous[i] + 
                                    left_fft_previous[i+1] * left_fft_previous[i+1];
            
            // Differenz (quadrierte Magnituden)
            float32_t diff = mag_curr_sq - mag_prev_sq;
            
            if(diff > 0.0f)
            {
                flux += sqrtf(diff);  // Nur 1x sqrt statt 2x
            }
        }
    }
    
    // Speichere komplette FFT
    arm_copy_f32(fft_current, left_fft_previous, BLOCK_SIZE * 2);
    
    features->spectral_flux = flux;
}

int main()
{
    // Initialize platform
    init_platform(115200, hz32000, line_in);
    
    // Init test pin
    gpio_set(TEST_PIN, LOW);
    
    // Initialize circular buffers
    rx_buffer.init();
    tx_buffer.init();
    memset(in, 0, sizeof(in));
    memset(out, 0, sizeof(out));
    
    // Initialize FFT instance
    arm_status status = arm_rfft_fast_init_f32(&fft_instance, BLOCK_SIZE);
    if(status != ARM_MATH_SUCCESS)
    {
        IF_DEBUG(debug_printf("FFT Init failed!\n"));
        fatal_error();
    }
    arm_hamming_f32(hamming_window, BLOCK_SIZE);

    // Initialisiere Flux Buffer
    memset(left_fft_previous, 0, sizeof(left_fft_previous));
    memset(right_fft_previous, 0, sizeof(right_fft_previous));
    first_flux_block = true;


    IF_DEBUG(debug_printf("fs: %.2f Hz, N: %d\n", fs, BLOCK_SIZE));
    IF_DEBUG(debug_printf("df: %.2f Hz, dt: %.2f ms\n",fs / BLOCK_SIZE,BLOCK_SIZE / fs * 1000.0));
    
    // Start I2S/DMA
    platform_start();
    
    while(true)
    {
        // Step 1: Read block of samples from input buffer
        while(!rx_buffer.read(in));
        
        gpio_set(LED_B, HIGH);      // LED_B off
        gpio_set(TEST_PIN, HIGH);   // Test Pin High
        
        // Step 2: Split samples into two channels
        convert_audio_sample_to_2ch(in, left_in, right_in);
        
        // Step 3: Convert from int16 to float32
        for(uint32_t n = 0; n < BLOCK_SIZE; n++)
        {
        left_float[n] = ((float32_t)left_in[n] / 32768.0f) * hamming_window[n];
        right_float[n] = ((float32_t)right_in[n] / 32768.0f) * hamming_window[n];     
       }
        
        // Step 4: Perform FFT on both channels
        arm_rfft_fast_f32(&fft_instance, left_float, left_fft, 0);   // 0 = forward FFT
        arm_rfft_fast_f32(&fft_instance, right_float, right_fft, 0);
        
        // Step 4.5: Calculate Flux DIREKT aus FFT (VOR Magnitude!)
        calculate_fft_flux_optimized(left_fft, &left_features);
        
        // Step 5: Calculate magnitude spectrum for left channel
        calculate_magnitude(left_fft, magnitude, BLOCK_SIZE);
        
        // Step 6: Extract features from left channel
        extract_features(magnitude, &left_features);
        
        // Step 7: Calculate magnitude spectrum for right channel
        calculate_magnitude(right_fft, magnitude, BLOCK_SIZE);
        
        // // Step 8: Extract features from right channel
        // extract_features(magnitude, &right_features);
        
        // Step 9: Output features (example - every 100th frame)
        static uint32_t frame_count = 0;
        frame_count++;
        if(frame_count % 5 == 0)
        {
            IF_DEBUG(debug_printf("SC: %.2f Hz\n", left_features.spectral_centroid));
            IF_DEBUG(debug_printf("P: %.2f\n", 10.0f * log10f(left_features.power)));
            IF_DEBUG(debug_printf("E: %.2f\n", 20.0f * log10f(left_features.energy)));
            IF_DEBUG(debug_printf("PF: %.2f Hz (Mag: %.2f)\n", left_features.peak_frequency, left_features.peak_magnitude));
            IF_DEBUG(debug_printf("Flux: %.6f\n", left_features.spectral_flux));

            // IF_DEBUG(debug_printf("\n=== RIGHT CHANNEL ===\n"));
            // IF_DEBUG(debug_printf("Spectral Centroid: %.2f Hz\n", right_features.spectral_centroid));
            // IF_DEBUG(debug_printf("Power: %.2f\n", right_features.power));
            // IF_DEBUG(debug_printf("Energy: %.2f\n", right_features.energy));
            // IF_DEBUG(debug_printf("Peak Frequency: %.2f Hz (Mag: %.2f)\n", 
            //          right_features.peak_frequency, right_features.peak_magnitude));
        }
        
        // Step 10: Pass through audio unchanged (copy input to output)
        for(uint32_t n = 0; n < BLOCK_SIZE; n++)
        {
            left_out[n] = left_in[n];
            right_out[n] = right_in[n];
        }
        
        // Step 11: Merge two channels into one sample
        convert_2ch_to_audio_sample(left_out, right_out, out);
        
        // Step 12: Write block of samples to output buffer
        while(!tx_buffer.write(out));
        
        gpio_set(LED_B, LOW);       // LED_B on
        gpio_set(TEST_PIN, LOW);    // Test Pin Low
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