from pydub import AudioSegment
import random
import csv
import os

# ===== EINSTELLUNGEN =====
TARGET_DURATION_MS = 30_000    # Ziel-Länge pro Spur (z.B. 10_000 für 10 s)
MIN_BLOCK_MS = 10             # minimale Blocklänge in ms
MAX_BLOCK_MS = 50             # maximale Blocklänge in ms
BLOCK_STEP_MS = 8             # Schrittweite für Blocklängen (Vielfaches)
OUTPUT_DIR = "Inputsignals/viel/"
BASE_OUTPUT_NAME = "interleaved_vio_8k_30sec_viel"

# HIER deine MP3-Dateien als 2D-Liste eintragen:
# Jede innere Liste = ein Paket, das zusammen interleaved wird
audio_files = [

    # Paket20: Musik Mix
    [
        "C:/eigene Programme/VS_Code_Programme/HKA/DSP/Raw_signals/sine_8khz.mp3",
        "C:/eigene Programme/VS_Code_Programme/HKA/DSP/Raw_signals/violin.mp3",
        "C:/eigene Programme/VS_Code_Programme/HKA/DSP/Raw_signals/sine_15khz.mp3",
    ],

    [
        "C:/eigene Programme/VS_Code_Programme/HKA/DSP/Raw_signals/sine_8khz.mp3",
        "C:/eigene Programme/VS_Code_Programme/HKA/DSP/Raw_signals/violin.mp3",
        "C:/eigene Programme/VS_Code_Programme/HKA/DSP/Raw_signals/sine_15khz.mp3",
        "C:/eigene Programme/VS_Code_Programme/HKA/DSP/Raw_signals/RAP_God.mp3",
    ],

    [
        "C:/eigene Programme/VS_Code_Programme/HKA/DSP/Raw_signals/sine_8khz.mp3",
        "C:/eigene Programme/VS_Code_Programme/HKA/DSP/Raw_signals/violin.mp3",
        "C:/eigene Programme/VS_Code_Programme/HKA/DSP/Raw_signals/sine_15khz.mp3",
        "C:/eigene Programme/VS_Code_Programme/HKA/DSP/Raw_signals/RAP_God.mp3",
        "C:/eigene Programme/VS_Code_Programme/HKA/DSP/Raw_signals/sine_500hz.mp3",
    ],



]



audio_paths = [


    "8k_vio_15k",
    "8k_vio_15k_Rap",
    "8k_vio_15k_Rap_500",


]


def load_and_normalize(path, target_duration_ms=TARGET_DURATION_MS):
    """
    Lädt eine Audiodatei, konvertiert sie auf Mono & einheitliche Samplerate
    und bringt sie per Kürzen/Loopen genau auf target_duration_ms.
    """
    audio = AudioSegment.from_file(path)

    # Auf Mono und feste Sample-Rate setzen (damit alles zusammenpasst)
    audio = audio.set_channels(1)
    audio = audio.set_frame_rate(44100)

    length = len(audio)

    if length > target_duration_ms:
        # Zu lang -> abschneiden
        audio = audio[:target_duration_ms]
    elif length < target_duration_ms:
        # Zu kurz -> wiederholen, bis die Ziel-Länge erreicht ist
        original = audio
        while len(audio) < target_duration_ms:
            remaining = target_duration_ms - len(audio)
            # nur so viel von original anhängen, wie noch fehlt
            audio += original[:remaining]

    return audio


def get_random_block_length(min_ms, max_ms, step_ms):
    """
    Gibt eine zufällige Blocklänge zurück, die ein Vielfaches von step_ms ist
    und zwischen min_ms und max_ms liegt.
    """
    # Berechne minimales und maximales Vielfaches
    min_multiple = int(min_ms / step_ms)
    max_multiple = int(max_ms / step_ms)
    
    # Wähle zufälliges Vielfaches
    random_multiple = random.randint(min_multiple, max_multiple)
    
    # Berechne tatsächliche Blocklänge
    block_ms = random_multiple * step_ms
    
    return block_ms


def interleave_segments(
    audio_list,
    min_block_ms=MIN_BLOCK_MS,
    max_block_ms=MAX_BLOCK_MS,
    block_step_ms=BLOCK_STEP_MS,
    target_duration_ms=TARGET_DURATION_MS
):
    """
    N normalisierte Audios werden in Segmente zufälliger Länge (min_block_ms–max_block_ms) zerlegt
    und streng interleaved zusammengebaut: A1,B1,C1,...,A2,B2,C2,...

    Für jeden "Durchlauf" (A1,B1,C1,...) wird eine neue zufällige Segmentlänge gewählt,
    die für alle Spuren gleich ist und ein Vielfaches von block_step_ms ist.
    """
    if not audio_list:
        raise ValueError("audio_list ist leer – füge Dateien in audio_files hinzu!")

    output = AudioSegment.silent(duration=0)
    pos = 0  # aktuelle Position in den Eingangs-Signalen
    change_events = []
    current_time_ms = 0
    seg_index = 0
    prev_label = None

    while pos < target_duration_ms:
        remaining = target_duration_ms - pos
        if remaining <= 0:
            break

        # Zufällige Blocklänge wählen (Vielfaches von block_step_ms)
        block_ms = get_random_block_length(min_block_ms, max_block_ms, block_step_ms)

        # Wenn der Block länger wäre als der Rest, auf Rest begrenzen
        if block_ms > remaining:
            block_ms = remaining

        start = pos
        end = pos + block_ms

        for src_idx, audio in enumerate(audio_list):
            # Label: A, B, C, ... (falls >26 Spuren -> S27, S28, ...)
            label = chr(ord('A') + src_idx) if src_idx < 26 else f"S{src_idx+1}"

            segment = audio[start:end]

            # Wechselstelle = Startzeit jedes Segments (außer beim allerersten)
            if seg_index > 0:
                change_events.append({
                    "change_ms": current_time_ms,
                    "change_s": current_time_ms / 1000.0,
                    "from": prev_label,
                    "to": label,
                    "segment_index": seg_index,
                    "block_ms": len(segment),
                    "source_pos_ms": start,
                })

            output += segment
            current_time_ms += len(segment)
            prev_label = label
            seg_index += 1

        pos += block_ms

    return output, change_events


def save_change_events_csv(events, csv_path):
    os.makedirs(os.path.dirname(csv_path) or ".", exist_ok=True)
    fieldnames = ["change_ms", "change_s", "from", "to", "segment_index", "block_ms", "source_pos_ms"]
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(events)


def main():
    # Stelle sicher, dass Output-Verzeichnis existiert
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    # Verarbeite jedes Paket
    for packet_idx, packet in enumerate(audio_files, start=1):
        print(f"\n{'='*60}")
        print(f"Verarbeite Paket {packet_idx} von {len(audio_files)}")
        print(f"{'='*60}")
        
        # 1. Alle Audios des Pakets laden und auf Ziel-Länge normalisieren
        prepared_audios = []
        for path in packet:
            print(f"Lade und normalisiere: {path}")
            prepared_audios.append(load_and_normalize(path))

        # 2. Interleaven mit zufälligen Blocklängen
        print("Interleave-Audio wird erstellt...")
        interleaved, change_events = interleave_segments(prepared_audios)

        # 3. Dateinamen für dieses Paket generieren
        output_file = os.path.join(OUTPUT_DIR, f"{audio_paths[packet_idx-1]}.mp3")
        csv_file = os.path.splitext(output_file)[0] + "_wechselstellen.csv"

        # 4. Speichern
        print(f"Speichere Ergebnis als: {output_file}")
        interleaved.export(output_file, format="mp3")
        
        # 5. Wechselstellen als CSV speichern
        #print(f"Speichere Wechselstellen als: {csv_file}")
        #save_change_events_csv(change_events, csv_file)
        
        #print(f"Paket {packet_idx} fertig!")
    
    print(f"\n{'='*60}")
    print(f"Alle {len(audio_files)} Pakete wurden erfolgreich verarbeitet!")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()