#include "global.h"
#include "arm_math.h"
#include "arm_const_structs.h"
 
CircularBuffer rx_buffer;
CircularBuffer tx_buffer;
 
uint32_t in[BLOCK_SIZE];
uint32_t out[BLOCK_SIZE];
int16_t left_in[BLOCK_SIZE];
int16_t right_in[BLOCK_SIZE];
int16_t left_out[BLOCK_SIZE];
int16_t right_out[BLOCK_SIZE];
 
// FFT Configuration - angepasst für Deinterleaving
#define FFT_SIZE 128  // Gute Balance: bei 48kHz = ~10.7ms, ausreichend für Frequenzauflösung
                      // Alternativen: 256 (5.3ms), 1024 (21.3ms) je nach Bedarf
 
// FFT Buffers für beide Kanäle
float32_t left_fft_input[FFT_SIZE * 2];   // *2 für Real/Imag
float32_t left_fft_output[FFT_SIZE * 2];
float32_t right_fft_input[FFT_SIZE * 2];
float32_t right_fft_output[FFT_SIZE * 2];
 
// Overlap-Add Buffer für kontinuierliche Verarbeitung
float32_t left_overlap[FFT_SIZE];
float32_t right_overlap[FFT_SIZE];
uint32_t overlap_index = 0;
 
// FFT Instance (verwende vordefinierte Strukturen für Effizienz)
arm_rfft_fast_instance_f32 fft_instance;
 
// Hanning Window für bessere Frequenzauflösung
float32_t window[FFT_SIZE];
 
// Hilfsfunktion: Hanning Window generieren
void generate_hanning_window() {
    for(uint32_t i = 0; i < FFT_SIZE; i++) {
        window[i] = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / (FFT_SIZE - 1)));
    }
}
 
int main()
{
    init_platform(115200, hz8000, line_in);
    debug_printf("%s, %s\n", __DATE__, __TIME__);
    IF_DEBUG(debug_printf("FFT Deinterleaving - FFT_SIZE: %d\n", FFT_SIZE));
   
    gpio_set(TEST_PIN, LOW);
   
    rx_buffer.init();
    tx_buffer.init();
    memset(in, 0, sizeof(in));
    memset(out, 0, sizeof(out));
   
    // FFT Initialisierung
    arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);
    generate_hanning_window();
   
    // Overlap Buffer initialisieren
    memset(left_overlap, 0, sizeof(left_overlap));
    memset(right_overlap, 0, sizeof(right_overlap));
    overlap_index = 0;
   
    platform_start();
   
    while(true)
    {
        while(!rx_buffer.read(in));
       
        gpio_set(LED_B, HIGH);
        gpio_set(TEST_PIN, HIGH);
       
        convert_audio_sample_to_2ch(in, left_in, right_in);
       
        // ========== STEP 3: FFT Processing ==========
       
        // 3.1: Konvertierung int16 -> float32 und Windowing
        for(uint32_t n = 0; n < BLOCK_SIZE; n++) {
            uint32_t idx = overlap_index + n;
            if(idx < FFT_SIZE) {
                // Normalisierung von int16 [-32768, 32767] zu float [-1.0, 1.0]
                left_overlap[idx] = ((float32_t)left_in[n]) / 32768.0f;
                right_overlap[idx] = ((float32_t)right_in[n]) / 32768.0f;
            }
        }
       
        overlap_index += BLOCK_SIZE;
       
        // Wenn genug Samples für FFT vorhanden sind
        if(overlap_index >= FFT_SIZE) {
            // Window anwenden und in FFT Input kopieren
            for(uint32_t i = 0; i < FFT_SIZE; i++) {
                left_fft_input[i] = left_overlap[i] * window[i];
                right_fft_input[i] = right_overlap[i] * window[i];
            }
           
            // 3.2: FFT durchführen
            arm_rfft_fast_f32(&fft_instance, left_fft_input, left_fft_output, 0);   // 0 = Forward FFT
            arm_rfft_fast_f32(&fft_instance, right_fft_input, right_fft_output, 0);
           
            // ===== HIER: FFT-Daten für Deinterleaving analysieren =====
            // left_fft_output und right_fft_output enthalten nun:
            // [0] = DC, [1] = Real(f1), [2] = Imag(f1), [3] = Real(f2), [4] = Imag(f2), ...
            // Frequenzauflösung: 48000Hz / 512 = 93.75 Hz pro Bin
           
            // Beispiel: Magnitudenberechnung für Frequenzanalyse
            float32_t left_magnitude[FFT_SIZE/2];
            float32_t right_magnitude[FFT_SIZE/2];
            arm_cmplx_mag_f32(left_fft_output, left_magnitude, FFT_SIZE/2);
            arm_cmplx_mag_f32(right_fft_output, right_magnitude, FFT_SIZE/2);
           
            // TODO: Deinterleaving-Logik basierend auf Frequenzinhalt
            // z.B.: Signaldetektion, Trennung von überlappenden Signalen, etc.
           
            // 3.3: Inverse FFT (für dieses Beispiel - passthrough)
            arm_rfft_fast_f32(&fft_instance, left_fft_output, left_fft_input, 1);   // 1 = Inverse FFT
            arm_rfft_fast_f32(&fft_instance, right_fft_output, right_fft_input, 1);
           
            // Normalisierung nach IFFT und Rückkonvertierung zu int16
            float32_t scale = 1.0f / (float32_t)FFT_SIZE;
            for(uint32_t i = 0; i < BLOCK_SIZE; i++) {
                left_out[i] = (int16_t)(left_fft_input[i] * scale * 32768.0f);
                right_out[i] = (int16_t)(right_fft_input[i] * scale * 32768.0f);
            }
           
            // Overlap-Buffer für nächsten Block vorbereiten
            uint32_t remaining = overlap_index - FFT_SIZE;
            if(remaining > 0) {
                memmove(left_overlap, &left_overlap[FFT_SIZE], remaining * sizeof(float32_t));
                memmove(right_overlap, &right_overlap[FFT_SIZE], remaining * sizeof(float32_t));
            }
            overlap_index = remaining;
           
        } else {
            // Noch nicht genug Samples, Output = Input (Passthrough)
            for(uint32_t n = 0; n < BLOCK_SIZE; n++) {
                left_out[n] = left_in[n];
                right_out[n] = right_in[n];
            }
        }
       
        // ========== END STEP 3 ==========
       
        convert_2ch_to_audio_sample(left_out, right_out, out);
       
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
    if(temp == nullptr) {
        fatal_error();
    }
    return temp;
}
 
uint32_t* get_new_rx_buffer_ptr()
{
    uint32_t* temp = rx_buffer.get_write_ptr();
    if(temp == nullptr) {
        fatal_error();
    }
    return temp;
}