/*
 * Author: Jan Eberhardt
 * Modified: Single channel FFT/IFFT processing
 */
#include "global.h"
#include "arm_math.h"
 
 
// FFT configuration - choose FFT size independently from BLOCK_SIZE
#define NFFT 128  // Must be power of 2: 32, 64, 128, 256, 512, 1024, 2048, 4096
 
// Circular buffers
CircularBuffer rx_buffer;
CircularBuffer tx_buffer;
 
// Audio buffers
uint32_t in[BLOCK_SIZE];
uint32_t out[BLOCK_SIZE];
 
// FFT buffers and structures
float32_t signal_float[NFFT];
float32_t signal_fft[NFFT * 2];   // Complex output (real + imag)
 
// ARM CMSIS DSP FFT instance
arm_rfft_fast_instance_f32 fft_instance;
 
int main()
{
    // Initialize platform
    init_platform(115200, hz48000, line_in);
   
    debug_printf("%s, %s\n", __DATE__, __TIME__);
    IF_DEBUG(debug_printf("Single Channel FFT/IFFT Audio Processing\n"));
   
    // Init test pin
    gpio_set(TEST_PIN, LOW);
   
    // Initialize circular buffers
    rx_buffer.init();
    tx_buffer.init();
    memset(in, 0, sizeof(in));
    memset(out, 0, sizeof(out));
   
    // Initialize FFT instance with chosen NFFT
    arm_status status = arm_rfft_fast_init_f32(&fft_instance, NFFT);
    if(status != ARM_MATH_SUCCESS)
    {
        IF_DEBUG(debug_printf("FFT Init failed!\n"));
        fatal_error();
    }
   
    IF_DEBUG(debug_printf("FFT initialized: BLOCK_SIZE=%d, NFFT=%d\n", BLOCK_SIZE, NFFT));
   
    // Start I2S/DMA
    platform_start();
   
    while(true)
    {
        // Step 1: Read block of samples from input buffer
        while(!rx_buffer.read(in));
       
        gpio_set(LED_B, HIGH);      // LED_B off
        gpio_set(TEST_PIN, HIGH);   // Test Pin High
       
        // Step 2: Process the audio signal with FFT/IFFT
       
        // 2.1: Convert from int to float32 and handle size mismatch
        if(NFFT >= BLOCK_SIZE)
        {
            // FFT is larger: copy data and zero-pad
            for(uint32_t n = 0; n < BLOCK_SIZE; n++)
            {
                signal_float[n] = (float32_t)((int16_t)(in[n] & 0xFFFF));
            }
            // Zero-pad the rest
            for(uint32_t n = BLOCK_SIZE; n < NFFT; n++)
            {
                signal_float[n] = 0.0f;
            }
        }
        else
        {
            // FFT is smaller: only use first NFFT samples
            for(uint32_t n = 0; n < NFFT; n++)
            {
                signal_float[n] = (float32_t)((int16_t)(in[n] & 0xFFFF));
            }
        }
       
        // 2.2: Perform FFT
        arm_rfft_fast_f32(&fft_instance, signal_float, signal_fft, 0);   // 0 = forward FFT
       
        // 2.3: Analyze frequency domain data
       
        float32_t total_power = 0.0f;
        float32_t total_energy = 0.0f;
        float32_t spectral_centroid_num = 0.0f;
        float32_t spectral_centroid_den = 0.0f;
        float32_t max_magnitude = 0.0f;
        uint32_t peak_bin = 0;
       
        // Calculate bin frequency resolution
        float32_t freq_resolution = 48000.0f / (float32_t)NFFT; // Hz per bin
       
        // Process FFT bins (skip DC [0] and Nyquist [1])
        for(uint32_t k = 1; k < NFFT/2; k++)
        {
            float32_t real = signal_fft[2*k];
            float32_t imag = signal_fft[2*k+1];
           
            // Calculate magnitude
            float32_t magnitude = sqrtf(real*real + imag*imag);
           
            // Calculate power (magnitude squared)
            float32_t power = magnitude * magnitude;
           
            // Accumulate total power
            total_power += power;
           
            // Accumulate for spectral centroid (weighted frequency average)
            float32_t frequency = (float32_t)k * freq_resolution;
            spectral_centroid_num += frequency * magnitude;
            spectral_centroid_den += magnitude;
           
            // Find peak (highest magnitude)
            if(magnitude > max_magnitude)
            {
                max_magnitude = magnitude;
                peak_bin = k;
            }
        }
       
        // Calculate total energy (sum of power)
        total_energy = total_power;
       
        // Calculate spectral centroid (center of mass of spectrum)
        float32_t spectral_centroid = 0.0f;
        if(spectral_centroid_den > 0.0f)
        {
            spectral_centroid = spectral_centroid_num / spectral_centroid_den;
        }
       
        // Calculate peak frequency
        float32_t peak_frequency = (float32_t)peak_bin * freq_resolution;
       
        // Calculate spectral rolloff (frequency below which 85% of energy is contained)
        float32_t energy_threshold = 0.85f * total_energy;
        float32_t cumulative_energy = 0.0f;
        uint32_t rolloff_bin = 0;
        for(uint32_t k = 1; k < NFFT/2; k++)
        {
            float32_t real = signal_fft[2*k];
            float32_t imag = signal_fft[2*k+1];
            float32_t magnitude = sqrtf(real*real + imag*imag);
            cumulative_energy += magnitude * magnitude;
           
            if(cumulative_energy >= energy_threshold)
            {
                rolloff_bin = k;
                break;
            }
        }
        float32_t spectral_rolloff = (float32_t)rolloff_bin * freq_resolution;
       
        // Calculate spectral bandwidth (standard deviation around centroid)
        float32_t spectral_bandwidth_num = 0.0f;
        for(uint32_t k = 1; k < NFFT/2; k++)
        {
            float32_t real = signal_fft[2*k];
            float32_t imag = signal_fft[2*k+1];
            float32_t magnitude = sqrtf(real*real + imag*imag);
            float32_t frequency = (float32_t)k * freq_resolution;
            float32_t diff = frequency - spectral_centroid;
            spectral_bandwidth_num += diff * diff * magnitude;
        }
        float32_t spectral_bandwidth = 0.0f;
        if(spectral_centroid_den > 0.0f)
        {
            spectral_bandwidth = sqrtf(spectral_bandwidth_num / spectral_centroid_den);
        }
       
        // Optional: Print analysis results (use sparingly to avoid slowing down real-time processing)
        // Uncomment for debugging:
        IF_DEBUG(debug_printf("Power: %.2f, Energy: %.2f, Centroid: %.1f Hz, Peak: %.1f Hz (%.2f), Rolloff: %.1f Hz, Bandwidth: %.1f Hz\n",
                              total_power, total_energy, spectral_centroid, peak_frequency, max_magnitude, spectral_rolloff, spectral_bandwidth));
       
        // Optional: Modify spectrum here (e.g., filtering, effects)
        // For now: Just pass through without modification
       
        // 2.4: Perform IFFT
        arm_rfft_fast_f32(&fft_instance, signal_fft, signal_float, 1);   // 1 = inverse FFT
       
        // 2.5: Convert from float32 back to int and handle size mismatch
        uint32_t samples_to_copy = (NFFT < BLOCK_SIZE) ? NFFT : BLOCK_SIZE;
       
        for(uint32_t n = 0; n < samples_to_copy; n++)
        {
            // Clip values to int16 range
            float32_t val = signal_float[n];
           
            if(val > 32767.0f) val = 32767.0f;
            if(val < -32768.0f) val = -32768.0f;
           
            out[n] = (uint32_t)((int16_t)val);
        }
       
        // If NFFT < BLOCK_SIZE, fill remaining samples with zeros
        for(uint32_t n = samples_to_copy; n < BLOCK_SIZE; n++)
        {
            out[n] = 0;
        }
       
        // Step 3: Write block of samples to output buffer
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
 