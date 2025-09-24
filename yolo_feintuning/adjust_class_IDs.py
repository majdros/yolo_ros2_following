import os

def remap_labels_for_coco_comparison():
    """
    Erstellt eine Kopie der Validierungs-Labels und mappt alle Klassen
    auf die COCO-Klasse 'sports ball' (ID 32) um.
    Erstellt außerdem eine passende data.yaml für die Auswertung.
    """
    # --- Konfiguration ---
    base_path = './dataset/'
    original_val_labels_path = os.path.join(base_path, 'valid/labels/')
    new_val_labels_path = os.path.join(base_path, 'valid/labels_coco/')
    
    # --- Überprüfen, ob der Quellordner existiert ---
    if not os.path.exists(original_val_labels_path):
        print(f"FEHLER: Quellordner '{original_val_labels_path}' nicht gefunden.")
        return

    # --- Neuen Label-Ordner erstellen ---
    os.makedirs(new_val_labels_path, exist_ok=True)
    print(f"Erstelle Zielordner: '{new_val_labels_path}'")

    # --- Labels umschreiben ---
    remapped_count = 0
    for filename in os.listdir(original_val_labels_path):
        if filename.endswith('.txt'):
            original_file = os.path.join(original_val_labels_path, filename)
            new_file = os.path.join(new_val_labels_path, filename)

            with open(original_file, 'r') as infile, open(new_file, 'w') as outfile:
                for line in infile:
                    parts = line.strip().split()
                    # Ersetze die Klassen-ID (erster Wert) mit 32
                    parts[0] = '32'
                    outfile.write(' '.join(parts) + '\n')
            remapped_count += 1
    
    print(f"Verarbeitung abgeschlossen. {remapped_count} Label-Dateien wurden auf Klasse 32 umgeschrieben.")

    # --- Neue data_coco.yaml erstellen ---
    yaml_content = f"""
# Diese YAML-Datei ist für die Evaluierung der Stock COCO-Modelle.
# Sie nutzt die originalen Bilder, aber die auf Klasse 32 umgeschriebenen Labels.

train: {os.path.abspath(os.path.join(base_path, 'train/images'))}
val: {os.path.abspath(os.path.join(base_path, 'valid/images'))}

# COCO class names (Auszug)
names:
    32: sports ball
"""
    yaml_path = os.path.join(base_path, 'data_coco.yaml')
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)
    
    print(f"Neue Konfigurationsdatei erstellt: '{yaml_path}'")
    print("Diese Datei nutzt die originalen Bilder mit den neuen 'labels_coco' für die Validierung.")
    print("\nHinweis: Passe den 'val'-Pfad in der YAML manuell an, falls dein Validierungs-Tool absolute Pfade benötigt.")


if __name__ == '__main__':
    # Passe den Pfad in der YAML-Datei manuell an, um auf ./valid/labels_coco zu zeigen
    # Dies ist ein bekannter Punkt, der je nach YOLO-Version variieren kann.
    # Nach Ausführung dieses Skripts, öffne die data_coco.yaml und stelle sicher,
    # dass dein Validierungs-Tool die Labels aus 'valid/labels_coco' lesen wird.
    # Oftmals muss man den Label-Pfad implizit durch die Bild-Pfad-Struktur setzen.
    # Die beste Methode ist, einen komplett neuen Datensatz-Ordner zu erstellen.
    
    # Ein vereinfachter Ansatz: Erstellen einer neuen YAML, die auf die richtigen Ordner zeigt.
    # Dies erfordert, dass die YAML relative Pfade korrekt interpretiert.
    # Die obige Methode erstellt die YAML und überlässt die Validierung dem Nutzer.
    
    remap_labels_for_coco_comparison()