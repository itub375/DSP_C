/*
 * Author: Jan Eberhardt
 * Extended with Signal Feature Extraction and 500ms Latency Buffer
 */
#include "global.h"
#include "arm_math.h"  // CMSIS-DSP library

// Circular buffers
CircularBuffer rx_buffer;
CircularBuffer tx_buffer;

// Audio I/O buffers
uint32_t in[BLOCK_SIZE];
uint32_t out[BLOCK_SIZE];
int16_t left_in[BLOCK_SIZE];
int16_t right_in[BLOCK_SIZE];
int16_t left_out[BLOCK_SIZE];
int16_t right_out[BLOCK_SIZE];

// FFT Configuration
#define FFT_SIZE 256                    // 8 ms bei 32 kHz
#define FFT_SIZE_HALF (FFT_SIZE/2)
#define SAMPLE_RATE 32000               // Integer für Array-Größen
#define SAMPLE_RATE_F 32000.0f          // Float für Berechnungen

// Latency buffer for 500ms delay
#define LATENCY_MS 500
#define LATENCY_SAMPLES (SAMPLE_RATE * LATENCY_MS / 1000)  // 16000 samples at 32kHz
#define LATENCY_BLOCKS (LATENCY_SAMPLES / BLOCK_SIZE)      // 125 blocks

int16_t delay_buffer_left[LATENCY_SAMPLES];
int16_t delay_buffer_right[LATENCY_SAMPLES];
uint32_t delay_write_index = 0;
uint32_t delay_read_index = 0;
uint32_t delay_fill_counter = 0;
bool delay_buffer_filled = false;

// FFT Buffers and Instance
arm_rfft_fast_instance_f32 fft_instance;
float32_t fft_input[FFT_SIZE];          // Time domain input
float32_t fft_output[FFT_SIZE];         // Complex FFT output
float32_t fft_magnitude[FFT_SIZE_HALF]; // Magnitude spectrum
float32_t window[FFT_SIZE];             // Hann window
float32_t prev_magnitude[FFT_SIZE_HALF]; // For flux calculation

// Feature accumulation buffer
float32_t audio_buffer[FFT_SIZE];
uint32_t buffer_index = 0;

// Feature results
float32_t centroid = 0.0f;
float32_t rms = 0.0f;
float32_t zcr = 0.0f;
float32_t flux = 0.0f;
float32_t bandwidth = 0.0f;

// Output control
uint32_t frame_counter = 0;
#define OUTPUT_EVERY_N_FRAMES 50  // Ausgabe alle ~400ms bei 32kHz und BLOCK_SIZE=128

// Function prototypes
void init_fft();
void create_hann_window();
void calculate_features(float32_t* samples, uint32_t length);
float32_t calculate_rms(float32_t* samples, uint32_t length);
float32_t calculate_zcr(float32_t* samples, uint32_t length);
void calculate_spectral_features();
void delay_buffer_write(int16_t* left, int16_t* right, uint32_t length);
void delay_buffer_read(int16_t* left, int16_t* right, uint32_t length);

int main()
{
    // Initialize platform
    init_platform(115200, hz32000, line_in);
    
    debug_printf("%s, %s\n", __DATE__, __TIME__);
    IF_DEBUG(debug_printf("Audio Signal Analysis Started\n"));
    IF_DEBUG(debug_printf("FFT Size: %d samples (%.1f ms)\n", FFT_SIZE, (float)FFT_SIZE/SAMPLE_RATE_F*1000.0f));
    IF_DEBUG(debug_printf("Latency Buffer: %d ms (%d samples)\n", LATENCY_MS, LATENCY_SAMPLES));
    
    // Init test pin
    gpio_set(TEST_PIN, LOW);
    
    // Initialize circular buffers
    rx_buffer.init();
    tx_buffer.init();
    memset(in, 0, sizeof(in));
    memset(out, 0, sizeof(out));
    
    // Initialize delay buffers
    memset(delay_buffer_left, 0, sizeof(delay_buffer_left));
    memset(delay_buffer_right, 0, sizeof(delay_buffer_right));
    
    // Initialize FFT and window
    init_fft();
    create_hann_window();
    
    // Initialize buffers
    memset(audio_buffer, 0, sizeof(audio_buffer));
    memset(prev_magnitude, 0, sizeof(prev_magnitude));
    
    // Start I2S/DMA
    platform_start();
    
    IF_DEBUG(debug_printf("Filling latency buffer...\n"));
    
    while(true)
    {
        // Step 1: Read block of samples
        while(!rx_buffer.read(in));
        
        gpio_set(LED_B, HIGH);
        gpio_set(TEST_PIN, HIGH);
        
        // Step 2: Split samples into two channels
        convert_audio_sample_to_2ch(in, left_in, right_in);
        
        // Step 3: Write input to delay buffer
        delay_buffer_write(left_in, right_in, BLOCK_SIZE);
        
        // Step 4: Audio processing and feature extraction (on input signal)
        // Convert left channel to float for analysis
        for(uint32_t n = 0; n < BLOCK_SIZE; n++)
        {
            // Accumulate samples into FFT buffer
            audio_buffer[buffer_index++] = (float32_t)left_in[n] / 32768.0f; // Normalize to [-1, 1]
            
            // When buffer is full, calculate features
            if(buffer_index >= FFT_SIZE)
            {
                buffer_index = 0;
                calculate_features(audio_buffer, FFT_SIZE);
                frame_counter++;
                
                // Output features periodically
                if(frame_counter >= OUTPUT_EVERY_N_FRAMES)
                {
                    frame_counter = 0;
                    debug_printf("\n=== Signal Features ===\n");
                    debug_printf("RMS:       %.4f\n", rms);
                    debug_printf("ZCR:       %.2f Hz\n", zcr);
                    debug_printf("Centroid:  %.2f Hz\n", centroid);
                    debug_printf("Bandwidth: %.2f Hz\n", bandwidth);
                    debug_printf("Flux:      %.4f\n", flux);
                }
            }
        }
        
        // Step 5: Read delayed audio from delay buffer
        delay_buffer_read(left_out, right_out, BLOCK_SIZE);
        
        // Step 6: Merge two channels
        convert_2ch_to_audio_sample(left_out, right_out, out);
        
        // Step 7: Write block to output buffer
        while(!tx_buffer.write(out));
        
        gpio_set(LED_B, LOW);
        gpio_set(TEST_PIN, LOW);
    }
    
    fatal_error();
    return 0;
}

void delay_buffer_write(int16_t* left, int16_t* right, uint32_t length)
{
    for(uint32_t i = 0; i < length; i++)
    {
        delay_buffer_left[delay_write_index] = left[i];
        delay_buffer_right[delay_write_index] = right[i];
        
        delay_write_index++;
        if(delay_write_index >= LATENCY_SAMPLES)
        {
            delay_write_index = 0;
        }
    }
    
    // Track when buffer is sufficiently filled
    if(!delay_buffer_filled)
    {
        delay_fill_counter++;
        if(delay_fill_counter >= LATENCY_BLOCKS)
        {
            delay_buffer_filled = true;
            IF_DEBUG(debug_printf("Latency buffer filled. Starting audio output.\n"));
        }
    }
}

void delay_buffer_read(int16_t* left, int16_t* right, uint32_t length)
{
    // Only read after buffer is filled, otherwise output silence
    if(!delay_buffer_filled)
    {
        memset(left, 0, length * sizeof(int16_t));
        memset(right, 0, length * sizeof(int16_t));
        return;
    }
    
    for(uint32_t i = 0; i < length; i++)
    {
        left[i] = delay_buffer_left[delay_read_index];
        right[i] = delay_buffer_right[delay_read_index];
        
        delay_read_index++;
        if(delay_read_index >= LATENCY_SAMPLES)
        {
            delay_read_index = 0;
        }
    }
}

void init_fft()
{
    // Initialize CMSIS-DSP Real FFT
    arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);
}

void create_hann_window()
{
    // Create Hann window: w(n) = 0.5 * (1 - cos(2*pi*n/(N-1)))
    for(uint32_t i = 0; i < FFT_SIZE; i++)
    {
        window[i] = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * (float32_t)i / (float32_t)(FFT_SIZE - 1)));
    }
}

void calculate_features(float32_t* samples, uint32_t length)
{
    // 1. Calculate RMS (time domain)
    rms = calculate_rms(samples, length);
    
    // 2. Calculate ZCR (time domain)
    zcr = calculate_zcr(samples, length);
    
    // 3. Apply window and prepare for FFT
    for(uint32_t i = 0; i < FFT_SIZE; i++)
    {
        fft_input[i] = samples[i] * window[i];
    }
    
    // 4. Perform FFT
    arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0);
    
    // 5. Calculate magnitude spectrum
    // FFT output format: [real0, imag0, real1, imag1, ...]
    // DC component
    fft_magnitude[0] = fabsf(fft_output[0]);
    
    // Other bins
    for(uint32_t i = 1; i < FFT_SIZE_HALF; i++)
    {
        float32_t real = fft_output[2*i];
        float32_t imag = fft_output[2*i + 1];
        fft_magnitude[i] = sqrtf(real*real + imag*imag);
    }
    
    // 6. Calculate spectral features
    calculate_spectral_features();
}

float32_t calculate_rms(float32_t* samples, uint32_t length)
{
    float32_t sum = 0.0f;
    
    for(uint32_t i = 0; i < length; i++)
    {
        sum += samples[i] * samples[i];
    }
    
    return sqrtf(sum / (float32_t)length);
}

float32_t calculate_zcr(float32_t* samples, uint32_t length)
{
    uint32_t zero_crossings = 0;
    
    for(uint32_t i = 1; i < length; i++)
    {
        // Count sign changes
        if((samples[i] >= 0.0f && samples[i-1] < 0.0f) || 
           (samples[i] < 0.0f && samples[i-1] >= 0.0f))
        {
            zero_crossings++;
        }
    }
    
    // Convert to rate (crossings per second)
    float32_t zcr_rate = (float32_t)zero_crossings / ((float32_t)length / SAMPLE_RATE_F);
    
    return zcr_rate;
}

void calculate_spectral_features()
{
    float32_t sum_magnitude = 0.0f;
    float32_t weighted_sum = 0.0f;
    float32_t flux_sum = 0.0f;
    
    // Frequency resolution
    float32_t freq_resolution = SAMPLE_RATE_F / (float32_t)FFT_SIZE;
    
    // Calculate total energy and weighted sum for centroid
    for(uint32_t i = 0; i < FFT_SIZE_HALF; i++)
    {
        float32_t magnitude = fft_magnitude[i];
        float32_t frequency = (float32_t)i * freq_resolution;
        
        sum_magnitude += magnitude;
        weighted_sum += magnitude * frequency;
        
        // Calculate flux (difference from previous frame)
        float32_t diff = magnitude - prev_magnitude[i];
        flux_sum += diff * diff;
        
        // Store for next frame
        prev_magnitude[i] = magnitude;
    }
    
    // 1. Spectral Centroid (Schwerpunkt)
    if(sum_magnitude > 0.0001f)
    {
        centroid = weighted_sum / sum_magnitude;
    }
    else
    {
        centroid = 0.0f;
    }
    
    // 2. Spectral Flux
    flux = sqrtf(flux_sum);
    
    // 3. Spectral Bandwidth (Standardabweichung um Centroid)
    float32_t variance = 0.0f;
    
    for(uint32_t i = 0; i < FFT_SIZE_HALF; i++)
    {
        float32_t frequency = (float32_t)i * freq_resolution;
        float32_t diff = frequency - centroid;
        variance += fft_magnitude[i] * diff * diff;
    }
    
    if(sum_magnitude > 0.0001f)
    {
        bandwidth = sqrtf(variance / sum_magnitude);
    }
    else
    {
        bandwidth = 0.0f;
    }
}

// DMA callbacks
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