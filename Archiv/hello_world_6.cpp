/*

* Author: Jan Eberhardt

* Modified: FFT/IFFT processing added

*/

#include "global.h"

#include "arm_math.h"
 
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
 
// ARM CMSIS DSP FFT instances

arm_rfft_fast_instance_f32 fft_instance;
 
int main()

{

    // Initialize platform

    init_platform(115200, hz48000, line_in);

    debug_printf("%s, %s\n", __DATE__, __TIME__);

    IF_DEBUG(debug_printf("FFT/IFFT Audio Processing\n"));

    // Init test pin

    gpio_set(TEST_PIN, LOW);

    // Initialize circular buffers

    rx_buffer.init();

    tx_buffer.init();

    memset(in, 0, sizeof(in));

    memset(out, 0, sizeof(out));

    // Initialize FFT instance

    // BLOCK_SIZE must be 32, 64, 128, 256, 512, 1024, 2048, or 4096

    arm_status status = arm_rfft_fast_init_f32(&fft_instance, BLOCK_SIZE);

    if(status != ARM_MATH_SUCCESS)

    {

        IF_DEBUG(debug_printf("FFT Init failed!\n"));

        fatal_error();

    }

    IF_DEBUG(debug_printf("FFT initialized with BLOCK_SIZE=%d\n", BLOCK_SIZE));

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

        // Step 3: Process the audio channels with FFT/IFFT

        // 3.1: Convert from int16 to float32

        for(uint32_t n = 0; n < BLOCK_SIZE; n++)

        {

            left_float[n] = (float32_t)left_in[n];

            right_float[n] = (float32_t)right_in[n];

        }

        // 3.2: Perform FFT on both channels

        arm_rfft_fast_f32(&fft_instance, left_float, left_fft, 0);   // 0 = forward FFT

        arm_rfft_fast_f32(&fft_instance, right_float, right_fft, 0);

        // 3.3: Optional: Process frequency domain data here

        // Example: Modify left_fft and right_fft arrays

        // Format: [real0, imag0, real1, imag1, ..., realN/2, imagN/2]

        // For now: Just pass through without modification

        // 3.4: Perform IFFT on both channels

        arm_rfft_fast_f32(&fft_instance, left_fft, left_float, 1);   // 1 = inverse FFT

        arm_rfft_fast_f32(&fft_instance, right_fft, right_float, 1);

        // 3.5: Convert from float32 back to int16

        for(uint32_t n = 0; n < BLOCK_SIZE; n++)

        {

            // Clip values to int16 range

            float32_t left_val = left_float[n];

            float32_t right_val = right_float[n];

            if(left_val > 32767.0f) left_val = 32767.0f;

            if(left_val < -32768.0f) left_val = -32768.0f;

            if(right_val > 32767.0f) right_val = 32767.0f;

            if(right_val < -32768.0f) right_val = -32768.0f;

            left_out[n] = (int16_t)left_val;

            right_out[n] = (int16_t)right_val;

        }

        // Step 4: Merge two channels into one sample

        convert_2ch_to_audio_sample(left_out, right_out, out);

        // Step 5: Write block of samples to output buffer

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
 