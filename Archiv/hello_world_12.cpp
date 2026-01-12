#include "global.h"
#include "arm_math.h"
 
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
 
// ===== FFT CONFIG =====
#define FFT_SIZE 256
#define NUM_CHANNELS 3
 
// ===== FFT ANALYZER =====
class FFTAnalyzer {
private:
    arm_rfft_fast_instance_f32 fft_instance;
    float32_t input_buf[FFT_SIZE];
    float32_t output_buf[FFT_SIZE];
    float32_t mag_buf[FFT_SIZE/2];
public:
    void init() {
        arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);
    }
    float32_t analyze(int16_t* input, uint32_t fs) {
        // Convert to float
        for(uint32_t i = 0; i < FFT_SIZE; i++) {
            input_buf[i] = (float32_t)input[i] / 32768.0f;
        }
        // FFT
        arm_rfft_fast_f32(&fft_instance, input_buf, output_buf, 0);
        arm_cmplx_mag_f32(output_buf, mag_buf, FFT_SIZE/2);
        // Find peak
        uint32_t peak_idx = 5;
        float32_t peak_val = 0;
        arm_max_f32(&mag_buf[5], FFT_SIZE/2 - 5, &peak_val, &peak_idx);
        peak_idx += 5;
        return (float32_t)peak_idx * fs / FFT_SIZE;
    }
};
 
// ===== GLOBALS =====
FFTAnalyzer g_fft;
int16_t g_ch_buf[NUM_CHANNELS][FFT_SIZE];
uint32_t g_buf_idx = 0;
int g_selected_ch = -1;
bool g_analyzed = false;
 
int main()
{
    init_platform(115200, hz32000, line_in);
    gpio_set(TEST_PIN, LOW);
    rx_buffer.init();
    tx_buffer.init();
    memset(in, 0, sizeof(in));
    memset(out, 0, sizeof(out));
    memset(g_ch_buf, 0, sizeof(g_ch_buf));
    g_fft.init();
    platform_start();
    while(true)
    {
        // Step 1
        while(!rx_buffer.read(in));
        gpio_set(LED_B, HIGH);
        gpio_set(TEST_PIN, HIGH);
        // Step 2
        convert_audio_sample_to_2ch(in, left_in, right_in);
        // Step 3: FFT & DEINTERLEAVE
        if(!g_analyzed) {
            // Collect samples
            for(uint32_t n = 0; n < BLOCK_SIZE && g_buf_idx < FFT_SIZE; n++) {
                g_ch_buf[0][g_buf_idx] = left_in[n];
                g_ch_buf[1][g_buf_idx] = right_in[n];
                g_ch_buf[2][g_buf_idx] = (left_in[n] + right_in[n]) >> 1;
                g_buf_idx++;
            }
            // Analyze when full
            if(g_buf_idx >= FFT_SIZE) {
                float32_t freq[NUM_CHANNELS];
                float32_t min_freq = 100000.0f;
                for(int ch = 0; ch < NUM_CHANNELS; ch++) {
                    freq[ch] = g_fft.analyze(g_ch_buf[ch], 32000);
                    if(freq[ch] < min_freq && freq[ch] > 100.0f) {
                        min_freq = freq[ch];
                        g_selected_ch = ch;
                    }
                }
                g_analyzed = true;
                g_buf_idx = 0;
            }
            // Passthrough during analysis
            for(uint32_t n = 0; n < BLOCK_SIZE; n++) {
                left_out[n] = left_in[n];
                right_out[n] = right_in[n];
            }
        }
        else {
            // Output selected channel
            for(uint32_t n = 0; n < BLOCK_SIZE; n++) {
                int16_t sample;
                switch(g_selected_ch) {
                    case 0: sample = left_in[n]; break;
                    case 1: sample = right_in[n]; break;
                    case 2: sample = (left_in[n] + right_in[n]) >> 1; break;
                    default: sample = 0; break;
                }
                left_out[n] = sample;
                right_out[n] = sample;
            }
        }
        // Step 4
        convert_2ch_to_audio_sample(left_out, right_out, out);
        // Step 5
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
    if(temp == nullptr) fatal_error();
    return temp;
}
 
uint32_t* get_new_rx_buffer_ptr()
{
    uint32_t* temp = rx_buffer.get_write_ptr();
    if(temp == nullptr) fatal_error();
    return temp;
}