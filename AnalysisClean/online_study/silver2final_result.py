import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import os

# --- CONFIGURAZIONE ---
BASE_PATH = os.path.dirname(__file__)
SILVER_PATH = os.path.join(BASE_PATH, 'silver_layer')
REPORT_PATH = os.path.join(BASE_PATH, 'report_finali')

def inizializza():
    os.makedirs(REPORT_PATH, exist_ok=True)
    sns.set_theme(style="whitegrid", context="paper")

def carica_e_normalizza():
    df = pd.read_csv(os.path.join(SILVER_PATH, 'dataset_unificato_pulito.csv'))
    # Filtro i due paesi principali
    df = df[df['Language'].isin(['Italiano', 'Deutsch'])].copy()
    
    # Identificazione colonne e normalizzazione (1-5 -> 0-1)
    likert_cols = [c for c in df.columns if 'Robot_' in c]
    df[likert_cols] = df[likert_cols].apply(pd.to_numeric, errors='coerce')
    df[likert_cols] = (df[likert_cols] - 1) / 4
    return df, likert_cols

def estrai_risultati_finali(df, cols):
    # Calcolo medie per lingua e prospettiva
    summary = df.groupby('Language')[cols].mean().transpose().reset_index()
    summary.columns = ['Domanda', 'DE', 'IT']
    
    # Separiamo per prospettiva
    summary['Prospettiva'] = summary['Domanda'].apply(lambda x: 'Third View (Human)' if 'Human' in x else 'First View (You)')
    summary['ID_Distanza'] = summary['Domanda'].str.extract(r'(Q\d)').iloc[:,0]
    
    return summary

def genera_grafico_confronto(summary):
    """Genera la curva prossemica per individuare il picco di distanza ottimale."""
    plt.figure(figsize=(10, 6))
    
    # Plot per prospettiva[cite: 1]
    for prosp in summary['Prospettiva'].unique():
        data_sub = summary[summary['Prospettiva'] == prosp]
        linestyle = '-' if 'First' in prosp else '--'
        
        plt.plot(data_sub['ID_Distanza'], data_sub['IT'], label=f'IT - {prosp}', 
                 marker='o', linestyle=linestyle, color='forestgreen', linewidth=2)
        plt.plot(data_sub['ID_Distanza'], data_sub['DE'], label=f'DE - {prosp}', 
                 marker='s', linestyle=linestyle, color='royalblue', linewidth=2)

    plt.title("Confronto delle Curve Prossemiche: Identificazione Distanza Ottimale", fontsize=14)
    plt.xlabel("ID Distanza (Q1 ravvicinata - Q6 distante)")
    plt.ylabel("Accettabilità Media (Normalizzata 0-1)")
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(REPORT_PATH, 'curva_prossemica_finale.pdf'))

def identifica_best_distance(summary):
    """Calcola matematicamente la distanza con il punteggio medio più alto tra i gruppi[cite: 1]."""
    summary['Media_Globale'] = (summary['IT'] + summary['DE']) / 2
    
    # Troviamo la distanza migliore per prospettiva[cite: 1]
    best = summary.sort_values('Media_Globale', ascending=False).groupby('Prospettiva').head(1)
    
    with open(os.path.join(REPORT_PATH, 'conclusione_sperimentale.txt'), 'w', encoding='utf-8') as f:
        f.write("=== CONCLUSIONE DELL'ESPERIMENTO PROSSEMICO ===\n\n")
        for _, row in best.iterrows():
            f.write(f"PROSPETTIVA: {row['Prospettiva']}\n")
            f.write(f"  Distanza Ottimale Identificata: {row['ID_Distanza']}\n")
            f.write(f"  Punteggio di Accettazione Medio: {row['Media_Globale']:.2f}\n")
            f.write(f"  Divergenza IT-DE (Gap): {abs(row['IT'] - row['DE']):.3f}\n")
            f.write("-" * 40 + "\n")
        
        f.write("\nNOTA METODOLOGICA:\n")
        f.write("- Se il Gap è < 0.1, la distanza è considerata universalmente valida.\n")
        f.write("- Se il punteggio è < 0.5, la distanza è percepita come 'non confortevole'.\n")

def main():
    inizializza()
    df, cols = carica_e_normalizza()
    summary = estrai_risultati_finali(df, cols)
    genera_grafico_confronto(summary)
    identifica_best_distance(summary)
    print(f"Risultati finali generati in: {REPORT_PATH}")

if __name__ == "__main__":
    main()