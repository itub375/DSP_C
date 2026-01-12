/*
 * Author: DSP Gruppe 3
 * Modified: Spectral feature extraction
 * Berechnet: Spectral Centroid, Power, Energy, Frequenzpeak
 * 
 * 4.0
 * 
 */
#include "global.h"
#include "arm_math.h"
#include <math.h>



// Wie viele Blöcke A in Ausgabe gepuffert werden
#define SIGNAL_ROWS 60

// typedef enum
// {
//     hz8000  = 0x0C,  // 8kHz from 12.288MHz MCLK
//     hz32000 = 0x18,  // 32kHz from 12.288MHz MCLK
// 	   hz48000 = 0x00,  // 48kHz from 12.288MHz MCLK
//     hz96000 = 0x1C,  // 96kHz from 12.288MHz MCLK
// } sampling_rate;

// CFAR Parameter für Start-Erkennung
#define CFAR_WINDOW_SIZE 20      // Anzahl der Blöcke für Mittelwertbildung
#define CFAR_GUARD_CELLS 2       // Guard cells (nicht in Mittelwert einbeziehen)
#define CFAR_THRESHOLD_FACTOR 10.0f  // Schwellwert-Faktor (anpassbar)

// CFAR Parameter für End-Erkennung
#define END_DETECTION_FRAMES 200     // Anzahl der Frames ohne Signal für End-Erkennung
#define END_ENERGY_THRESHOLD_FACTOR 0.3f  // Schwellwert relativ zur durchschnittlichen Signal-Energie


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Allgemeine FFT Parameter
float32_t fs = 32000.0f;
float32_t df = fs / float32_t(BLOCK_SIZE); // Delta f
float32_t dT = 1.0f/df;                    // Delta T für FFT Block

// Parameter für feature Gewichtung
uint16_t weightCentroid = 2;
uint16_t weightPower = 2;
uint16_t weightEnergy = 2;
uint16_t weightpeakF = 1;
uint16_t weightFlux = 3;

float32_t score = 0.0f;
float32_t score_threshold = 0.3f;

uint16_t threshCentroid = 200;
uint16_t threshPower = 3;
uint16_t threshEnergy = 5;
uint16_t threshpeakF = 300;
uint16_t threshFlux = 2;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CFAR Variablen
float32_t energy_history[CFAR_WINDOW_SIZE] = {0.0f};
uint32_t energy_history_index = 0;
uint32_t energy_history_count = 0;
bool signal_detected = false;
int Signal_count = 0;
int min_Signal_count=50;  // Warum 50 ? weils funktioniert (unter 20 gehts nicht)

// Zusätzliche Variablen für End-Erkennung
uint32_t low_energy_counter = 0;     // Zählt Frames mit niedriger Energie
float32_t signal_energy_reference = 0.0f;  // Referenz-Energie des erkannten Signals
bool reference_energy_set = false;    // Flag ob Referenz-Energie gesetzt wurde

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
float32_t left_fftPast[BLOCK_SIZE * 2];
float32_t right_fftPast[BLOCK_SIZE * 2];
float32_t left_float[BLOCK_SIZE];
float32_t right_float[BLOCK_SIZE];

// Ausgangssignal zusammengesetzt
float32_t signalAusgang[SIGNAL_ROWS][BLOCK_SIZE*2] = {0};
uint32_t write_index = 0;

// Feature extraction buffers
float32_t magnitude[BLOCK_SIZE / 2];  // Magnitude spectrum
float32_t magnitude_past[BLOCK_SIZE / 2];   

// ARM CMSIS DSP FFT instances
arm_rfft_fast_instance_f32 FFT_conf;

// Fensterfunktion (by default Rechteck)
float32_t window[BLOCK_SIZE] = {1.0f};

// Feature results
typedef struct {
    // Merkmale des aktuellen FFT Blocks
    float32_t spectral_centroid = 0.0f;
    float32_t power = 0.0f;
    float32_t energy = 0.0f;
    float32_t peak_frequency = 0.0f;
    float32_t peak_magnitude = 0.0f;
    float32_t flux = 0.0f;

    // Merkmale des vorherigen FFT Blocks für Signalzuordnung
    float32_t spectral_centroid_past = 0.0f;
    float32_t power_past = 0.0f;
    float32_t energy_past = 0.0f;
    float32_t peak_frequency_past = 0.0f;
    float32_t peak_magnitude_past = 0.0f;
    float32_t flux_past = 0.0f;
} AudioFeatures;

AudioFeatures left_features;
AudioFeatures right_features;
AudioFeatures signalA;

////////////////////////////

// Zusätzliche globale Variable für Ausgabe-State
uint32_t playback_index = 0;      // Welcher Block wird gerade ausgegeben
bool is_playing = false;           // Wird gerade ein gespeichertes Signal ausgegeben?


// Funktion zur Berechnung der Magnitude aus FFT
void calculate_magnitude(float32_t* fft_data, float32_t* mag, uint32_t fft_size)
{
    mag[0] = fabsf(fft_data[0]); // dc
    // FFT output format: [real0, imag0, real1, imag1, ..., realN/2, imagN/2]
    // Berechne Magnitude für jede Frequenz bin
    for(uint32_t i = 1; i < fft_size / 2; i++)
    {
        // Realteil = fft_data[2*i];
        // Imaginärteil = fft_data[2*i+1];
        mag[i] = sqrtf(fft_data[2*i] * fft_data[2*i] + fft_data[2*i+1] * fft_data[2*i+1]);
    }
   
   // mag[1]=0.f;
}

float32_t magnitude_sum_past = 0.0f;

// Funktion zur Berechnung der Audio Features
void extract_features(float32_t* mag, float32_t* mag_past, AudioFeatures* features)
{
    float32_t weighted_sum = 0.0f;
    float32_t magnitude_sum = 0.0f;
    float32_t power_sum = 0.0f;
    float32_t peak_mag = 0.0f;
    uint32_t peak_index = 0;
    float32_t flux_sum = 0.0f; 

    // Merkmale des vorherigen FFT Blocks merken
    features->spectral_centroid_past = features->spectral_centroid;
    features->power_past = features->power;
    features->energy_past = features->energy;
    features->peak_frequency_past = features->peak_frequency;
    features->peak_magnitude_past = features->peak_magnitude;
    features->flux_past = features->flux;

    // --- 1. PRE-CALCULATE sum (thresholded) ---
    for(uint32_t i = 1; i < BLOCK_SIZE / 2; i++)
    {
        if (mag[i] > 0.01f)
            magnitude_sum += mag[i];
    }

    // --- 2. CALCULATE features ---
    for(uint32_t i = 1; i < BLOCK_SIZE / 2; i++)
    {        
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

    magnitude_sum_past = magnitude_sum;

    features->spectral_centroid = (magnitude_sum > 1e-6f) ? (weighted_sum / magnitude_sum) : 0.0f;
    features->energy = power_sum;
    features->power = 10.0f * log10f(power_sum / (BLOCK_SIZE / 2.0f));
    features->peak_frequency = peak_index * df;
    features->peak_magnitude = peak_mag;
    features->flux = flux_sum;
}

double normalize(double value, double min, double max) {

    double n = (value - min) / (max - min);
    
    if (n < 0.0) return 0.0f;

    if (n > 1.0) return 1.0f;

    return n;

}

// CFAR Detektor
bool cfar_detector(float32_t current_energy)
{
    // Energie in Ringpuffer speichern
    energy_history[energy_history_index] = current_energy;
    energy_history_index = (energy_history_index + 1) % CFAR_WINDOW_SIZE;
    
    if(energy_history_count < CFAR_WINDOW_SIZE)
    {
        energy_history_count++;
    }
    
    // Wenn noch nicht genug Daten: kein Signal detektieren
    if(energy_history_count < CFAR_WINDOW_SIZE)
    {
        return false;
    }
    
    // Mittelwert der Energie berechnen (ohne Guard Cells)
    float32_t sum = 0.0f;
    uint32_t count = 0;
    
    for(uint32_t i = 0; i < CFAR_WINDOW_SIZE; i++)
    {
        // Guard cells überspringen (die letzten CFAR_GUARD_CELLS Werte)
        int32_t distance = (int32_t)energy_history_index - (int32_t)i - 1;
        if(distance < 0) distance += CFAR_WINDOW_SIZE;
        
        if(i >= CFAR_GUARD_CELLS)
        {
            sum += energy_history[distance];
            count++;
        }
    }
    
    float32_t mean_energy = (count > 0) ? (sum / count) : 0.0f;
    
    // Schwellwert berechnen
    float32_t threshold = mean_energy * CFAR_THRESHOLD_FACTOR;
    
    // Debug output (optional)
    static uint32_t debug_count = 0;
    debug_count++;
    if(debug_count % 50 == 0)
    {
        IF_DEBUG(debug_printf("CFAR: Energy=%.2f, Mean=%.2f, Threshold=%.2f\n", 
                 current_energy, mean_energy, threshold));
    }
    
    // Signal detektieren wenn aktuelle Energie über Schwellwert
    return (current_energy > threshold);
}

// Funktion zur End-Erkennung
bool check_signal_end(float32_t current_energy)
{
    // Wenn noch keine Referenz-Energie gesetzt wurde, setze sie
    if(!reference_energy_set && signal_detected)
    {
        signal_energy_reference = current_energy;
        reference_energy_set = true;
        low_energy_counter = 0;
        return false;  // Signal läuft noch
    }
    
    // Berechne Schwellwert basierend auf Referenz-Energie
    float32_t end_threshold = signal_energy_reference * END_ENERGY_THRESHOLD_FACTOR;
    
    // Prüfe ob aktuelle Energie unter Schwellwert liegt
    if(current_energy < end_threshold)
    {
        low_energy_counter++;
        
        // Debug output (optional)
        static uint32_t debug_count = 0;
        debug_count++;
        if(debug_count % 50 == 0)
        {
            IF_DEBUG(debug_printf("END CHECK: Energy=%.2f, Ref=%.2f, Threshold=%.2f, Counter=%u\n", 
                     current_energy, signal_energy_reference, end_threshold, low_energy_counter));
        }
        
        // Wenn zu lange niedrige Energie: Signal ist beendet
        if(low_energy_counter >= END_DETECTION_FRAMES)
        {
            IF_DEBUG(debug_printf("SIGNAL ENDE erkannt nach %u Frames\n", low_energy_counter));
            return true;  // Signal ist beendet
        }
    }
    else
    {
        // Energie wieder über Schwellwert: Reset Counter
        low_energy_counter = 0;
        // Aktualisiere Referenz-Energie (gleitender Durchschnitt)
        signal_energy_reference = 0.9f * signal_energy_reference + 0.1f * current_energy;
    }
    
    return false;  // Signal läuft noch
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
    arm_status status = arm_rfft_fast_init_f32(&FFT_conf, BLOCK_SIZE);

    // Fensterfunktionsauswahl
    arm_hamming_f32(window, BLOCK_SIZE);
    // arm_hanning_f32(window, BLOCK_SIZE);

    // Einmaliger Print von den FFT Parametern
    IF_DEBUG(debug_printf("fs: %.2f Hz, N: %d\n", fs, BLOCK_SIZE));
    IF_DEBUG(debug_printf("df: %.2f Hz, dt: %.2f ms\n", df, dT * 1000.0));
    
    // Start I2S/DMA
    platform_start();
    //count = 0;
    signal_detected = false;
    
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
            left_float[n] = ((float32_t)left_in[n] / 32768.0f) * window[n];
            //right_float[n] = ((float32_t)right_in[n] / 32768.0f) * window[n];     
        }

    // Step 4: Perform FFT on both channels
        arm_rfft_fast_f32(&FFT_conf, left_float, left_fft, 0);   // 0 = forward FFT
        //arm_rfft_fast_f32(&FFT_conf, right_float, right_fft, 0);
        
    // Step 5: Calculate magnitude spectrum for left channel
        calculate_magnitude(left_fft, magnitude, BLOCK_SIZE);

    // Step 6: Extract features from left channel
        extract_features(magnitude, magnitude_past, &left_features);
        memcpy(magnitude_past, magnitude, sizeof(magnitude));

    // Step 7: CFAR Signal Detection

        // Nur weiter verarbeiten wenn Signal detektiert wurde
        if(!signal_detected)
        {
            signal_detected = cfar_detector(left_features.energy);
            // Kein Signal: Silence ausgeben
            memset(left_out, 0, sizeof(left_out));
            memset(right_out, 0, sizeof(right_out));
            
            // Step 11: Merge two channels into one sample
            convert_2ch_to_audio_sample(left_out, right_out, out);
            static uint32_t frame_count = 0;
            frame_count++;

            //IF_DEBUG(debug_printf("Signal nicht erkannt. Signal Counter: %i\n" , Signal_count));
            // Step 12: Write block of samples to output buffer
            while(!tx_buffer.write(out));
            
            gpio_set(LED_B, LOW);       // LED_B on
            gpio_set(TEST_PIN, LOW);    // Test Pin Low
            
            continue;  // Zurück zum Anfang der Schleife
        }
        
    // Step 8: Erstmaliges einlesen von Signal A
        // Signal wurde erkannt
        if(signal_detected == true && Signal_count < min_Signal_count){
            if(Signal_count ==(min_Signal_count-1)){
            IF_DEBUG(debug_printf("Signal erkannt. Erste Features eingeschrieben\n"));
            signalA.energy = left_features.energy;
            signalA.power = left_features.power;
            signalA.spectral_centroid = left_features.spectral_centroid;
            signalA.peak_frequency = left_features.peak_frequency;
            signalA.peak_magnitude = left_features.peak_magnitude;
            signalA.flux = left_features.flux;
            
            IF_DEBUG(debug_printf("SC: %.2f Hz\n", signalA.spectral_centroid));
            IF_DEBUG(debug_printf("P: %.2f dB\n", signalA.power));
            IF_DEBUG(debug_printf("E: %.2f\n", signalA.energy));
            IF_DEBUG(debug_printf("PF: %.2f Hz (Mag: %.2f)\n", signalA.peak_frequency, signalA.peak_magnitude));
            IF_DEBUG(debug_printf("F: %.2f\n", signalA.flux));
            IF_DEBUG(debug_printf("count: %i\n", Signal_count+1));        

            IF_DEBUG(debug_printf("\n"));
            }
            Signal_count += 1;
            //IF_DEBUG(debug_printf("\n"));
            //IF_DEBUG(debug_printf("\n"));
            //IF_DEBUG(debug_printf("\n"));
            
        }
        
    // Step 9: End-Erkennung falls signal verschwindet oder wieder NULL Anliegt
        // NEU: Prüfe auf Signal-Ende
        if(signal_detected && Signal_count >= min_Signal_count)
        {
            if(check_signal_end(left_features.energy))
            {
                // Signal ist beendet - Reset aller Variablen
                signal_detected = false;
                reference_energy_set = false;
                low_energy_counter = 0;
                Signal_count = 0;
                write_index = 0;
                
                IF_DEBUG(debug_printf("=== SIGNAL KOMPLETT BEENDET - System Reset ===\n"));
                
                // Silence ausgeben
                memset(left_out, 0, sizeof(left_out));
                memset(right_out, 0, sizeof(right_out));
                convert_2ch_to_audio_sample(left_out, right_out, out);
                while(!tx_buffer.write(out));
                
                gpio_set(LED_B, LOW);
                gpio_set(TEST_PIN, LOW);
                
                continue;
            }
        }

        // Step 10: Calculate magnitude spectrum for right channel
        // calculate_magnitude(right_fft, magnitude, BLOCK_SIZE);
        
        // Step 11: Extract features from right channel
        // extract_features(magnitude, &right_features);
        


    // Step 12 Detection of Signal A
        // Wenn score groß ist, dann Block A in left vorhanden
        score = 0.0f;

        // Centroid
        if(fabs(left_features.spectral_centroid - signalA.spectral_centroid) < threshCentroid) {
            score += weightCentroid*(1.0f-normalize(fabs(left_features.spectral_centroid-signalA.spectral_centroid), 0 , threshCentroid));
        }

        // Power
        if(fabs(left_features.power - signalA.power) < threshPower) {
            score += weightPower*(1.0f-normalize(fabs(left_features.power- signalA.power), 0, threshPower)); 
        }

        // Energy
        if(fabs(left_features.energy - signalA.energy) < threshEnergy) {
            score += weightEnergy*(1.0f-normalize(fabs(left_features.energy- signalA.energy), 0, threshEnergy));
        }

        // Peak Freq
        if(fabs(left_features.peak_frequency - signalA.peak_frequency) < threshpeakF) {
            score += weightpeakF*(1.0f-normalize(fabs(left_features.peak_frequency- signalA.peak_frequency), 0, threshpeakF));
        }

        // Flux
        if(fabs(left_features.flux - signalA.flux) < threshFlux) {
            score += weightFlux*(1.0f-normalize(fabs(left_features.flux- signalA.flux), 0 ,threshFlux));
        }

        score = score/(weightCentroid + weightPower + weightEnergy + weightpeakF + weightFlux);


    // Step 13: Output features (example - every 100th frame)
        static uint32_t frame_count = 0;
        frame_count++;
        if(frame_count % 100 == 0)
        {
            //IF_DEBUG(debug_printf("SC: %.2f Hz\n", left_features.spectral_centroid));
            //IF_DEBUG(debug_printf("P: %.2f dB\n", left_features.power));
            //IF_DEBUG(debug_printf("E: %.2f\n", left_features.energy));
            //IF_DEBUG(debug_printf("PF: %.2f Hz (Mag: %.2f)\n", left_features.peak_frequency, left_features.peak_magnitude));
            //IF_DEBUG(debug_printf("F: %.2f\n", left_features.flux-left_features.flux_past));
            IF_DEBUG(debug_printf("Score: %.2f\n", score));
            //IF_DEBUG(debug_printf("\n"));
            // IF_DEBUG(debug_printf("\n=== RIGHT CHANNEL ===\n"));
            // IF_DEBUG(debug_printf("Spectral Centroid: %.2f Hz\n", right_features.spectral_centroid));
            // IF_DEBUG(debug_printf("Power: %.2f\n", right_features.power));
            // IF_DEBUG(debug_printf("Energy: %.2f\n", right_features.energy));
            // IF_DEBUG(debug_printf("Peak Frequency: %.2f Hz (Mag: %.2f)\n", right_features.peak_frequency, right_features.peak_magnitude));
            IF_DEBUG(debug_printf("\n"));
        }

         


            
    // Step 14: erkennung ob Neues Signal anliegt durch Score
            if (score > score_threshold and Signal_count == min_Signal_count)
            {
                // 256 Werte von left nach signalA kopieren
                memcpy(signalAusgang[write_index], left_fft, BLOCK_SIZE * 2 * sizeof(float32_t));

                // Wenn A wieder erkannt wird features überschreiben
                signalA.energy = left_features.energy;
                signalA.power = left_features.power;
                signalA.spectral_centroid = left_features.spectral_centroid;
                signalA.peak_frequency = left_features.peak_frequency;
                signalA.peak_magnitude = left_features.peak_magnitude;
                signalA.flux = left_features.flux;

                if(frame_count % 100 == 0)
                {
                // IF_DEBUG(debug_printf("SC: %.2f Hz\n", signalA.spectral_centroid));
                // IF_DEBUG(debug_printf("P: %.2f dB\n", signalA.power));
                // IF_DEBUG(debug_printf("E: %.2f\n", signalA.energy));
                // IF_DEBUG(debug_printf("PF: %.2f Hz (Mag: %.2f)\n", signalA.peak_frequency, signalA.peak_magnitude));
                // IF_DEBUG(debug_printf("F: %.2f\n", signalA.flux));
                // IF_DEBUG(debug_printf("count: %i\n", Signal_count+1));  
                }

                




                            // Increment ring buffer index
                write_index++;
                if(write_index >= SIGNAL_ROWS)
                {                
                    IF_DEBUG(debug_printf("SignalA Komplett - starte Wiedergabe\n"));
                    write_index = 0;
                    playback_index = 0;
                    is_playing = true;
                }

            }

            // Step 15: Ausgabe - entweder gespeichertes Signal oder Stille
            if(is_playing)
            {
                // IFFT vom aktuellen Block
                arm_rfft_fast_f32(&FFT_conf, signalAusgang[playback_index], left_float, 1);  // 1 = inverse FFT
                
                for(uint32_t i = 0; i < BLOCK_SIZE; i++)
                {
                    float32_t sample = left_float[i];
                    // Clip to [-1.0, 1.0] to avoid overflow
                    if(sample > 1.0f) sample = 1.0f;
                    if(sample < -1.0f) sample = -1.0f;

                    int16_t out_sample = (int16_t)(sample * 32767.0f);

                    left_out[i] = out_sample;
                    right_out[i] = out_sample;  // mono
                }
                
                // Nächster Block für nächstes Frame
                playback_index++;
                if(playback_index >= SIGNAL_ROWS)
                {
                    IF_DEBUG(debug_printf("Wiedergabe beendet\n"));
                    playback_index = 0;
                    is_playing = false;  // Wiedergabe beendet
                }
            }
            else
            {
                // Stille während wir sammeln oder warten
                memset(left_out, 0, sizeof(left_out));
                memset(right_out, 0, sizeof(right_out));
            }



        
    // Step 15: Merge two channels into one sample
        convert_2ch_to_audio_sample(left_out, right_out, out);
        
    // Step 16: Write block of samples to output buffer
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