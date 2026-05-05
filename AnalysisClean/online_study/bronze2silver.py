import pandas as pd
import os

def pulisci_e_trasforma_dati(input_csv_path: str, output_silver_dir: str = 'silver_layer'):
    """
    Estrae e consolida i dati da un form multilingua, mantenendo la segmentazione 
    per lingua in ogni tabella e nelle relative statistiche.
    """
    
    # Inizializzazione directory
    os.makedirs(output_silver_dir, exist_ok=True)
    
    # Caricamento del dataset raw
    df_raw = pd.read_csv(input_csv_path)
    
    # Creazione DataFrame pulito
    df_clean = pd.DataFrame()
    
    # 1. Estrazione Lingua (Colonna all'indice 1)
    df_clean['Language'] = df_raw.iloc[:, 1]
    
    # Funzione di supporto per consolidamento orizzontale
    def consolida_colonne(indici):
        return df_raw.iloc[:, indici].bfill(axis=1).iloc[:, 0]

    # 2. Estrazione Prolific ID
    df_clean['Prolific_ID'] = consolida_colonne([4, 22, 40])
    
    # 3. Estrazione Argomento: Cultura
    df_clean['Culture_Country'] = consolida_colonne([5, 23, 41])
    df_clean['Culture_Language'] = consolida_colonne([6, 24, 42])
    df_clean['Culture_Culture_Self'] = consolida_colonne([7, 25, 43])
    
    # 4. Estrazione Robot-Human Distance (Video)
    for i in range(6):
        df_clean[f'Robot_Human_Q{i+1}'] = consolida_colonne([8+i, 26+i, 44+i])
        
    # 5. Estrazione Robot-You Perspective Distance
    for i in range(6):
        df_clean[f'Robot_You_Q{i+1}'] = consolida_colonne([14+i, 32+i, 50+i])
        
    # Rimozione righe prive di ID (dati incompleti)
    df_clean.dropna(subset=['Prolific_ID'], inplace=True)

    # --- FASE DI ESPORTAZIONE E STATISTICHE STRATIFICATE ---

    # Funzione interna per salvare dataset e statistiche divise per lingua
    def esporta_segmento(df_segmento, nome_base):
        # 1. Salvataggio dati completi del segmento (inclusa colonna Language)
        path_completo = os.path.join(output_silver_dir, f'{nome_base}.csv')
        df_segmento.to_csv(path_completo, index=False)
        
        # 2. Generazione statistiche per ogni lingua presente
        per_lingua_list = []
        for lingua in df_segmento['Language'].unique():
            # Filtra per lingua e rimuove ID per il conteggio
            subset = df_segmento[df_segmento['Language'] == lingua].drop(columns=['Prolific_ID', 'Language'], errors='ignore')
            
            # Calcola le frequenze dei valori (come nel file originale)
            stats = subset.apply(lambda x: x.value_counts()).fillna(0)
            stats.insert(0, 'Language', lingua)
            per_lingua_list.append(stats)
        
        # Unifica le statistiche in un unico file comparativo
        df_stats_finale = pd.concat(per_lingua_list).reset_index().rename(columns={'index': 'Valore_Risposta'})
        path_stats = os.path.join(output_silver_dir, f'statistiche_{nome_base}.csv')
        df_stats_finale.to_csv(path_stats, index=False)
        print(f"Esportati: {nome_base}.csv e statistiche_{nome_base}.csv")

    # Definizione dei segmenti da esportare (includendo sempre Language)
    
    # Segmento Cultura
    cult_cols = ['Prolific_ID', 'Language', 'Culture_Country', 'Culture_Language', 'Culture_Culture_Self']
    esporta_segmento(df_clean[cult_cols], 'argomento_cultura')

    # Segmento Robot-Human Video
    rh_cols = ['Prolific_ID', 'Language'] + [f'Robot_Human_Q{i+1}' for i in range(6)]
    esporta_segmento(df_clean[rh_cols], 'argomento_robot_human_video')

    # Segmento Robot-You Perspective
    ry_cols = ['Prolific_ID', 'Language'] + [f'Robot_You_Q{i+1}' for i in range(6)]
    esporta_segmento(df_clean[ry_cols], 'argomento_robot_you_perspective')

    # Salvataggio dataset unificato finale
    df_clean.to_csv(os.path.join(output_silver_dir, 'dataset_unificato_pulito.csv'), index=False)
    print("Processo completato: dataset_unificato_pulito.csv generato.")

if __name__ == "__main__":
    CSV_PATH = r"./bronze_layer/online_study.csv"
    pulisci_e_trasforma_dati(CSV_PATH, 'silver_layer')