#include "global.h"
#include "arm_math.h"
#include <math.h>

/*
================================================================================
DE: Was macht dieser Code?
    - Liest kontinuierlich Audio (Stereo, 32 kHz) per I2S/DMA in Blöcken (BLOCK_SIZE).
    - Bildet für die Analyse 50% überlappende Frames (HOP_SIZE = BLOCK_SIZE/2)
      und wendet ein Hamming-Fenster an.
    - Berechnet eine FFT (CMSIS-DSP), daraus Magnituden und Audio-Features
      (Centroid, Bandwidth, Rolloff, Power/Energy, Peak, Flux).
    - Start-Erkennung: CFAR-Detektor auf der Energie erkennt, wann "Signal" beginnt.
    - Referenzlernen: Die ersten min_Signal_count Frames nach Start dienen als
      Referenz "signalA" (Feature-Mittelwerte).
    - Laufender Betrieb: Pro Frame wird ein Score berechnet, wie ähnlich der Frame
      zu signalA ist. Wenn Score > score_threshold: Frame wird gespeichert.
    - RAM-Optimierung: FFT-Frames werden vor dem Speichern von float32 nach int16
      komprimiert (compress_fft). Für die Ausgabe wirdückwandlung (decompress_fft)
      und IFFT -> Zeitbereich.
    - End-Erkennung: Bei dauerhaft niedriger Energie (oder sehr kleinem Centroid)
      wird das Ende erkannt, State wird zurückgesetzt.

EN: What does this code do?
    - Continuously reads stereo audio (32 kHz) via I2S/DMA in blocks (BLOCK_SIZE).
    - Builds 50% overlapped analysis frames (HOP_SIZE = BLOCK_SIZE/2) and applies
      a Hamming window.
    - Computes an FFT (CMSIS-DSP), derives magnitudes and audio features
      (centroid, bandwidth, rolloff, power/energy, peak, flux).
    - Start detection: CFAR energy detector decides when a signal starts.
    - Reference learning: The first min_Signal_count frames after start are used
      to learn the reference "signalA" (averaged features).
    - Runtime: For each frame a similarity score to signalA is computed.
      If score > score_threshold: store the frame.
    - RAM optimization: FFT frames are compressed from float32 to int16 before
      storing (compress_fft). For playback they are decompressed and IFFT
      converts back to time-domain.
    - End detection: After a long low-energy period (or very small centroid),
      the end is detected and the state is reset.
================================================================================
*/

// *** GEÄNDERT: Doppelte Buffer-Größe durch Kompression ***
// DE: Anzahl gespeicherter Frames (Ringpuffer) – mehr = längere Aufnahmezeit (hier per Kompression verdoppelt)
// EN: Number of stored frames (ring buffer) – more = longer capture time (doubled via compression)

#define SIGNAL_ROWS 160  // War 80, jetzt 160 = ~5.12 Sekunden statt 2.56s

// DE: CFAR (Constant False Alarm Rate) – adaptiver Schwellwert basierend auf Energie-Historie
// EN: CFAR (Constant False Alarm Rate) – adaptive threshold based on recent energy history
// CFAR Parameter für Start-Erkennung
#define CFAR_WINDOW_SIZE 20
#define CFAR_GUARD_CELLS 2
#define CFAR_THRESHOLD_FACTOR 10.0f

// CFAR Parameter für End-Erkennung
#define END_DETECTION_FRAMES 100
#define END_ENERGY_THRESHOLD_FACTOR 0.3f

// DE: Overlap-Processing: 50% Overlap erzeugt glattere Feature-Folgen und bessere Zeitauflösung
// EN: Overlap processing: 50% overlap gives smoother features and better time resolution
// Overlap-Parameter
#define HOP_SIZE (BLOCK_SIZE / 2)  // 50% Overlap
#define OVERLAP_SIZE (BLOCK_SIZE - HOP_SIZE)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DE: Konfiguration – diese Werte bestimmen Erkennung/Scoring
// EN: Configuration – these values control detection/scoring
// Konfigurierbare Parameter
uint16_t Amount_of_Signals = 3;

// Allgemeine FFT Parameter
float32_t fs = 32000.0f;
float32_t df = fs / float32_t(BLOCK_SIZE); // DE: Frequenzauflösung (Hz pro Bin) | EN: Frequency resolution (Hz per bin)
float32_t dT = 1.0f/df; // DE: Zeitauflösung im Frequenzraster (≈ Frame-Zeit) | EN: Time scale derived from df

// Parameter für feature Gewichtung
uint16_t weightCentroid = 3;
uint16_t weightPower = 2;
uint16_t weightEnergy = 1;
uint16_t weightpeakF = 2;
uint16_t weightFlux = 2;
uint16_t         p: alter Overlap-Teil + neue HOP-Samples -> FFT-Frame

 = 3;

uint16_t weightRolloff = 1;


float32_t score = 0.0f;
float32_t score_threshold = 0.6f;

uint16_t threshCentroid = 1000;
uint16_t threshPower = 10;
uint16_t threshEnergy = 20;
uint16_t threshpeakF = 1733;
uint16_t threshFlux = 1;
uint16_t threshBandwidth = 1000;
uint16_t threshRolloff = 800;



uint16_t flux_counter = 0;



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// DE: Konfiguration – diese Werte bestimmen Erkennung/Scoring

// EN: Configuration – these values control detection/scoring



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

// DE: Konfiguration – diese Werte bestimmen Erkennung/Scoring

// EN: Configuration – these values control detection/scoring





// DE: DMA/I2S Circular Buffers – rx: Input, tx: Output

// EN: DMA/I2S circular buffers – rx: input, tx: output

// Circular buffers

CircularBuffer rx_buffer;

CircularBuffer tx_buffer;



// DE: Audio-Blockpuffer: interleaved 32-bit -> getrennte 16-bit Kanäle

// EN: Audio block buffers: interleaved 32-bit -> split into 16-bit channels

// Audio buffers

uint32_t in[BLOCK_SIZE];

uint32_t out[BLOCK_SIZE];

int16_t left_in[BLOCK_SIZE];

int16_t right_in[BLOCK_SIZE];

int16_t left_out[BLOCK_SIZE];

int16_t right_out[BLOCK_SIZE];



// DE: Overlap-Buffer speichert die zweite Hälfte des letzten Blocks für 50% Overlap

// EN: Overlap buffer stores the second half of the previous block for 50% overlap

// Overlap-Buffer für kontinuierliche Verarbeitung

float32_t overlap_buffer[OVERLAP_SIZE] = {0.0f};

bool overlap_initialized = false;





// DE: FFT-Daten: komplexes Spektrum (real+imag interleaved)

// EN: FFT data: complex spectrum (real+imag interleaved)

// FFT buffers and structures

float32_t left_fft[BLOCK_SIZE * 2] = {0.0f};

float32_t right_fft[BLOCK_SIZE * 2] = {0.0f};

float32_t left_fftPast[BLOCK_SIZE * 2] = {0.0f};

float32_t right_fftPast[BLOCK_SIZE * 2] = {0.0f};

float32_t left_float[BLOCK_SIZE];

float32_t right_float[BLOCK_SIZE];



// Temporärer Buffer für FFT-Input

float32_t fft_input_buffer[BLOCK_SIZE];



// DE: RAM-Sparen: Speichere FFT als int16 statt float32 (Skalierung in compress_fft)

// EN: RAM saving: store FFT as int16 instead of float32 (scaling in compress_fft)

// *** GEÄNDERT: Komprimierter Ausgangspuffer (16-bit statt 32-bit float) ***

// Spart 50% RAM: 160 Blöcke × 2048 int16 × 2 Bytes = 655 KB statt 1.31 MB

int16_t signalAusgang_compressed[SIGNAL_ROWS][BLOCK_SIZE*2] = {0};

uint32_t write_index = 0;



// *** NEU: Temporärer Dekompression-Buffer ***

float32_t decompression_buffer[BLOCK_SIZE * 2];



// Feature extraction buffers

float32_t magnitude[BLOCK_SIZE / 2];

float32_t magnitude_past[BLOCK_SIZE / 2] = {0.0f};



// ARM CMSIS DSP FFT instances

arm_rfft_fast_instance_f32 FFT_conf;



// Fensterfunktion

float32_t window[BLOCK_SIZE] = {1.0f};



// Feature results



// DE: Feature-Struktur (aktuell + *_past als Vorframe-Werte)

// EN: Feature struct (current + *_past = previous frame values)

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

AudioFeatures right_features;

AudioFeatures signalA;



// Ausgabe-State

uint32_t playback_index = 0;

bool is_playing = false;



// *** NEU: Kompression/Dekompression Funktionen ***



// Komprimiert FFT-Daten von float32 zu int16



// DE: Komprimiert FFT-Werte: float32 -> int16 (mit Skalierung + Clipping)

// EN: Compress FFT values: float32 -> int16 (scaling + clipping)

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



// DE: Dekomprimiert FFT-Werte: int16 -> float32 (inverse Skalierung)

// EN: Decompress FFT values: int16 -> float32 (inverse scaling)

void decompress_fft(int16_t* fft_int16, float32_t* fft_float, uint32_t length)

{

    const float32_t INV_SCALE = 1.0f / 16384.0f;

    

    for(uint32_t i = 0; i < length; i++)

    {

        fft_float[i] = (float32_t)fft_int16[i] * INV_SCALE;

    }

}



// Funktion zur Berechnung der Magnitude aus FFT



// DE: Berechnet Magnitude |X(k)| aus komplexem FFT-Output (real/imag)

// EN: Computes magnitude |X(k)| from complex FFT output (real/imag)

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



// DE: Feature-Extraktion aus Magnitudenspektrum (eine Frame)

// EN: Feature extraction from magnitude spectrum (one frame)

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



    // DE: Hauptschleife über Frequenz-Bins (1..K-1). Hier werden Summen/Peaks/Flux berechnet.

    // EN: Main loop over frequency bins (1..K-1). Accumulates sums/peaks/flux metrics.

    // DE: mag[i] ist Betragsspektrum; df ist Hz pro Bin.

    // EN: mag[i] is magnitude spectrum; df is Hz per bin.



    // DE: Hauptschleife über Frequenzbins (ohne DC-Bin 0):

    //     - magnitude_sum: Summe der Beträge (für Normierung/Schwellen)

    //     - weighted_sum: Summe (f * |X|) für Spektralzentrum

    //     - peak_mag/index: stärkster Bin (Peak-Frequenz)

    //     - power_sum: Summe |X|^2 (Energie/Power)

    //     - flux_sum: Änderung zu mag_past (Spektralfluss)

    //     - f_min/f_max: -3dB Bandbreite um den Peak (hier grob über Peak*1/sqrt(2))

    // EN: Main loop over frequency bins (excluding DC bin 0):

    //     - magnitude_sum: sum of magnitudes (normalization/thresholding)

    //     - weighted_sum: sum (f * |X|) for spectral centroid

    //     - peak_mag/index: strongest bin (peak frequency)

    //     - power_sum: sum |X|^2 (energy/power proxy)

    //     - flux_sum: change versus mag_past (spectral flux)

    //     - f_min/f_max: approx -3 dB bandwidth around the peak

    for(uint32_t i = 1; i < K; i++)

    {

        if (mag[i] > 0.01f) {magnitude_sum += mag[i];}

        if(mag[i] > 0.01f) {weighted_sum += (i * df) * mag[i];}



        if(mag[i] > peak_mag) {

            peak_mag = mag[i];

            peak_index = i;

        }



        power_sum += mag[i] * mag[i];  // DE: Energie/Power im Spektrum (Summe der Quadrate) | EN: spectral energy/power (sum of squares)



        if(fabs(mag[i] - mag_past[i]) > 0.0f) { 

            flux_sum += (mag[i] - mag_past[i]) * (mag[i] - mag_past[i]);  // DE: Spektralfluss (Frame-to-Frame Änderung) | EN: spectral flux (frame-to-frame change)

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

    float32_t thr = 0.85f * magnitude_sum; // DE: Rolloff bei 85% der Spektralenergie | EN: rolloff at 85% spectral energy





    // DE: Rolloff-Bestimmung: finde die kleinste Frequenz, bei der die akkumulierte Summe >= 85% der Gesamtsumme ist.

    // EN: Rolloff: find the smallest frequency where the accumulated sum reaches >= 85% of the total sum.

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

    // DE: Hinweis: flux wird hier als flux_sum (nicht normalisiert) gespeichert.

    // EN: Note: flux is stored as raw flux_sum here (not normalized).

    features->flux = flux_sum;

}





// DE: Normiert value in [0..1] basierend auf min/max und clippt

// EN: Normalizes value to [0..1] given min/max and clips

double normalize(double value, double min, double max) {

    double n = (value - min) / (max - min);

    

    if (n < 0.0) return 0.0;

    if (n > 1.0) return 1.0;

    

    return n;

}





// DE: CFAR-Startdetektor: vergleicht aktuelle Energie gegen adaptiven Schwellwert

// EN: CFAR start detector: compares current energy against adaptive threshold

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

    float32_t threshold = mean_energy * CFAR_THRESHOLD_FACTOR; // DE: adaptiver CFAR-Schwellwert | EN: adaptive CFAR threshold

    

    return (current_energy > threshold);

}





// DE: Ende-Erkennung: Energie fällt dauerhaft unter Referenz*Faktor (oder centroid ~0)

// EN: End detection: energy stays below reference*factor (or centroid ~0)

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





// DE: Hauptprogramm: Initialisierung + Endlos-Loop (read -> analyze -> detect -> store/play -> write)

// EN: Main program: init + infinite loop (read -> analyze -> detect -> store/play -> write)

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

    IF_DEBUG(debug_printf("Buffer: %d rows = %.2f sec (compressed)\n", SIGNAL_ROWS, 

             (SIGNAL_ROWS * HOP_SIZE) / fs));

    

    platform_start();

    signal_detected = false;

    

    while(true)

    {

        // DE: Schritt 1 – Blockweise Samples aus dem RX-Ringpuffer lesen (DMA -> rx_buffer)

        // EN: Step 1 – Read one block of samples from RX ring buffer (DMA -> rx_buffer)

        while(!rx_buffer.read(in));

        

        gpio_set(LED_B, HIGH);

        gpio_set(TEST_PIN, HIGH);

        

        // DE: Schritt 2 – Stereo-Samples in getrennte Left/Right-Arrays aufteilen

        // EN: Step 2 – Split interleaved stereo samples into left/right arrays

        convert_audio_sample_to_2ch(in, left_in, right_in);

        

        // DE: Schritt 3 – 50% Overlap: alter Overlap-Teil + neue HOP-Samples -> FFT-Frame

        // EN: Step 3 – 50% overlap: previous overlap + new hop samples -> FFT analysis frame

        for(uint32_t n = 0; n < OVERLAP_SIZE; n++)

        {

            fft_input_buffer[n] = overlap_buffer[n];

        }

        

        for(uint32_t n = 0; n < HOP_SIZE; n++)

        {

            fft_input_buffer[OVERLAP_SIZE + n] = (float32_t)left_in[n] / 32768.0f;

        }

        

        for(uint32_t n = 0; n < OVERLAP_SIZE; n++)

        {

            overlap_buffer[n] = (float32_t)left_in[HOP_SIZE + n] / 32768.0f;

        }

        

        for(uint32_t n = 0; n < BLOCK_SIZE; n++)

        {

            left_float[n] = fft_input_buffer[n] * window[n];

        }



        // DE: Schritt 4 – FFT im Frequenzbereich (Real FFT)

        // EN: Step 4 – FFT to frequency domain (real FFT)

        arm_rfft_fast_f32(&FFT_conf, left_float, left_fft, 0);

        

        // DE: Schritt 5 – Betragsspektrum |X(k)| berechnen

        // EN: Step 5 – Compute magnitude spectrum |X(k)|

        calculate_magnitude(left_fft, magnitude, BLOCK_SIZE);



        // DE: Schritt 6 – Audio-Features aus dem Spektrum extrahieren (Centroid, Bandwidth, …)

        // EN: Step 6 – Extract audio features from spectrum (centroid, bandwidth, …)

        extract_features(magnitude, magnitude_past, &left_features);

        memcpy(magnitude_past, magnitude, sizeof(magnitude));



        // DE: Schritt 7 – Startdetektion per CFAR auf Energie; bis dahin Stille ausgeben

        // EN: Step 7 – Start detection via CFAR on energy; output silence until detected

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

        

        // DE: Schritt 8 – Referenzlernen: nach min_Signal_count Frames signalA-Features initialisieren

        // EN: Step 8 – Reference learning: initialize signalA features after min_Signal_count frames

        if(signal_detected && Signal_count < min_Signal_count)

        {

            if(Signal_count == (min_Signal_count - 1))

            {

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

        

        // DE: Schritt 9 – Ende erkennen: dauerhaft niedrige Energie/kleiner Centroid -> Reset

        // EN: Step 9 – End detection: sustained low energy/small centroid -> reset state

        if(signal_detected && Signal_count >= min_Signal_count)

        {

            if(check_signal_end(left_features.energy, left_features.spectral_centroid))

            {

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



        // DE: Schritt 10 – Similarity-Score berechnen (Feature-Abstand -> normalisiert -> gewichtet)

        // EN: Step 10 – Compute similarity score (feature distance -> normalized -> weighted)

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



        score = score / (weightCentroid + weightPower + weightEnergy + weightpeakF + weightFlux + weightbandwidth + weightRolloff); // DE: Score auf [0..1] normalisieren | EN: Normalize score to ~[0..1]



        // DE: Schritt 11 – Wenn erkannt: FFT-Frame speichern (komprimiert), signalA adaptiv mitteln

        // EN: Step 11 – If detected: store FFT frame (compressed), update signalA by averaging

        if (score > score_threshold && Signal_count == min_Signal_count)

        {

            // *** GEÄNDERT: FFT-Daten komprimieren vor dem Speichern ***

            compress_fft(left_fft, signalAusgang_compressed[write_index], BLOCK_SIZE * 2);



            signalA.energy = (left_features.energy + signalA.energy) / 2.0f;

            signalA.power = (left_features.power + signalA.power) / 2.0f;

            signalA.spectral_bandwidth = (left_features.spectral_bandwidth + signalA.spectral_bandwidth) / 2.0f;

            signalA.spectral_rolloff = (left_features.spectral_rolloff + signalA.spectral_rolloff) / 2.0f;

            signalA.spectral_centroid = (left_features.spectral_centroid + signalA.spectral_centroid) / 2.0f;

            signalA.peak_frequency = (left_features.peak_frequency + signalA.peak_frequency) / 2.0f;

            signalA.peak_magnitude = (left_features.peak_magnitude + signalA.peak_magnitude) / 2.0f;

            signalA.flux = (left_features.flux + signalA.flux) / 2.0f;



            write_index++;

            if(write_index >= SIGNAL_ROWS)

            {                

                write_index = 0;

                playback_index = 0;

                is_playing = true;

            }

        }



        // DE: Schritt 12 – Playback: dekomprimieren -> IFFT -> Clip -> int16 -> Stereo ausgeben

        // EN: Step 12 – Playback: decompress -> IFFT -> clip -> int16 -> output stereo

        if(is_playing)

        {

            // *** GEÄNDERT: FFT-Daten dekomprimieren vor IFFT ***

            decompress_fft(signalAusgang_compressed[playback_index], decompression_buffer, BLOCK_SIZE * 2);

            

            arm_rfft_fast_f32(&FFT_conf, decompression_buffer, left_float, 1);

            

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

                playback_index = 0;

                is_playing = false;

            }

        }

        else

        {

            memset(left_out, 0, sizeof(left_out));

            memset(right_out, 0, sizeof(right_out));

        }



        // DE: Schritt 13 – Left/Right wieder zu interleavtem Audio-Format zusammenführen

        // EN: Step 13 – Merge left/right back to interleaved audio samples

        convert_2ch_to_audio_sample(left_out, right_out, out);

        

        // DE: Schritt 14 – In TX-Ringpuffer schreiben (DMA liest tx_buffer)

        // EN: Step 14 – Write to TX ring buffer (DMA drains tx_buffer)

        while(!tx_buffer.write(out));

        

        gpio_set(LED_B, LOW);

        gpio_set(TEST_PIN, LOW);

    }

    

    fatal_error();

    return 0;

}





// DE: DMA-Callback Helper: liefert Pointer auf naechsten TX-Block (zu senden)

// EN: DMA callback helper: returns pointer to next TX block (to transmit)

uint32_t* get_new_tx_buffer_ptr()

{

    uint32_t* temp = tx_buffer.get_read_ptr();

    if(temp == nullptr)

    {

        fatal_error();

    }

    return temp;

}





// DE: DMA-Callback Helper: liefert Pointer auf naechsten RX-Block (zu fuellen)

// EN: DMA callback helper: returns pointer to next RX block (to fill)

uint32_t* get_new_rx_buffer_ptr()

{

    uint32_t* temp = rx_buffer.get_write_ptr();

    if(temp == nullptr)

    {

        fatal_error();

    }

    return temp;

}