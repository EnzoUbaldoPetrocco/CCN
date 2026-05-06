import pandas as pd
import numpy as np
import os

# --- CONFIGURAZIONE PERCORSI ---
BASE_PATH = os.path.dirname(__file__)
SILVER_PATH = os.path.join(BASE_PATH, 'silver_layer')
# Nuova cartella per i risultati definitivi
FINAL_PATH = os.path.join(BASE_PATH, 'final_results_sep')

def inizializza_cartella():
    """Crea la cartella per i risultati finali se non esiste."""
    if not os.path.exists(FINAL_PATH):
        os.makedirs(FINAL_PATH)

def estrai_distanze_ottimali():
    """
    Analizza i dati normalizzati per identificare la distanza preferita 
    da ogni gruppo culturale nelle diverse prospettive.
    """
    path_dataset = os.path.join(SILVER_PATH, 'dataset_unificato_pulito.csv')
    df = pd.read_csv(path_dataset)
    
    # Filtro lingue target
    df = df[df['Language'].isin(['Italian', 'German'])].copy()
    
    # Colonne relative alle distanze[cite: 1]
    cols_robot = [c for c in df.columns if 'View' in c]
    df[cols_robot] = df[cols_robot].apply(pd.to_numeric, errors='coerce')
    
    # Normalizzazione 1-7 -> 0-1[cite: 1]
    df[cols_robot] = (df[cols_robot] - 1) / 6
    
    # Calcolo medie per lingua[cite: 1]
    medie_per_lingua = df.groupby('Language')[cols_robot].mean()
    
    report_linee = []
    report_linee.append("=== DETERMINAZIONE DISTANZE OTTIMALI PER CULTURA ===\n")
    
    for lingua in ['Italian', 'German']:
        report_linee.append(f"\nNATIOAL CULTURE: {lingua}")
        # Dividiamo per prospettiva[cite: 1]
        for vista in ['Video', 'Image']:
            label_vista = "3rd View (Video)" if vista == 'Video' else "1st View (Image)"
            filtro_cols = [c for c in cols_robot if vista in c]
            
            # Identificazione Q con media massima[cite: 1]
            best_q_col = medie_per_lingua.loc[lingua, filtro_cols].idxmax()
            best_val = medie_per_lingua.loc[lingua, best_q_col]
            
            # Estrazione solo del nome della domanda (es. Q3)[cite: 1]
            q_name = best_q_col.split('_')[-1]
            
            report_linee.append(f"  - {label_vista}: Distanza Ottimale = {q_name} (Rating: {best_val:.2f})")
    
    # Salvataggio del report testuale[cite: 1]
    with open(os.path.join(FINAL_PATH, 'sintesi_distanze.txt'), 'w', encoding='utf-8') as f:
        f.write("\n".join(report_linee))
    
    # Salvataggio delle medie in formato CSV per utilizzi futuri[cite: 1]
    medie_per_lingua.to_csv(os.path.join(FINAL_PATH, 'medie_normalizzate_finali.csv'))

def main():
    try:
        inizializza_cartella()
        estrai_distanze_ottimali()
        print(f"Processo completato. Risultati disponibili in: {FINAL_PATH}")
    except Exception as e:
        print(f"Errore durante l'esecuzione: {e}")

if __name__ == "__main__":
    main()