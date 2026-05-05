import pandas as pd
import numpy as np
import os

def process_silver_layer(input_path: str, base_output_dir: str = 'silver_layer'):
    """
    Segmenta il dataset in tre topic (Cultura, Personalità, Fiducia Robot) 
    ed esporta analisi statistiche dedicate per ciascuno.
    """
    
    # 1. CARICAMENTO E SETUP
    df = pd.read_csv(input_path)
    
    # Definizione delle sottocartelle
    topics = {
        'cultura': ['Which picture best describes your relationship with Italy or Germany?',
                    'Which picture best describes your relationship with Italian or German language?',
                    'Which picture best describes your relationship with Italian or German Culture?'],
        'personalita': [col for col in df.columns if "I see myself as someone who" in col],
        'robot_trust': [col for col in df.columns if "robot" in col.lower()]
    }

    # Mapping scale Likert per analisi quantitativa
    likert_map = {
        'Disagree strongly': 1,
        'Disagree a little': 2,
        'Neither agree or disagree': 3,
        'Agree a little': 4,
        'Agree strongly': 5
    }

    # 2. TRASFORMAZIONE E PULIZIA DATI
    # Pulizia percentuali nel gruppo robot_trust
    for col in topics['robot_trust']:
        df[col] = pd.to_numeric(df[col].astype(str).str.replace('%', ''), errors='coerce')
    
    # Mapping Likert nel gruppo personalita
    for col in topics['personalita']:
        df[col] = df[col].map(likert_map)

    # 3. ELABORAZIONE PER TOPIC
    for topic_name, columns in topics.items():
        # Creazione sottocartella specifica
        topic_dir = os.path.join(base_output_dir, topic_name)
        os.makedirs(topic_dir, exist_ok=True)
        
        # Selezione subset di dati (+ Nationality per stratificazione)
        subset_cols = ['Nationality'] + columns
        df_topic = df[subset_cols].copy()
        
        # Esportazione dataset pulito del topic
        df_topic.to_csv(os.path.join(topic_dir, f'data_{topic_name}.csv'), index=False)
        
        # --- ANALISI STATISTICA ---
        
        # Statistiche Globali
        stats_global = df_topic[columns].describe().transpose()
        stats_global.to_csv(os.path.join(topic_dir, f'stats_globali_{topic_name}.csv'))
        
        # Statistiche Stratificate per Cultura (Nationality)
        # Calcoliamo Media, Deviazione Standard e Mediana per ogni gruppo
        stats_by_culture = df_topic.groupby('Nationality')[columns].agg(['mean', 'std', 'median'])
        stats_by_culture.to_csv(os.path.join(topic_dir, f'stats_per_cultura_{topic_name}.csv'))
        
        print(f"Topic '{topic_name}' elaborato con successo in: {topic_dir}")

    # Esportazione dataset silver integrale per riferimento
    df.to_csv(os.path.join(base_output_dir, 'full_silver_dataset.csv'), index=False)

if __name__ == "__main__":
    # Percorso del file bronze in input
    INPUT_CSV = './bronze_layer/Intro CCN  (Risposte).CSV' 
    
    try:
        process_silver_layer(INPUT_CSV)
        print("\nPipeline Silver completata. Analisi disponibili nelle sottocartelle.")
    except Exception as e:
        print(f"Errore critico durante l'esecuzione: {e}")