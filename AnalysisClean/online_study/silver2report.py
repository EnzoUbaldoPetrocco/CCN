import pandas as pd
import numpy as np
import os
from scipy import stats
import matplotlib.pyplot as plt
import seaborn as sns

# --- CONFIGURAZIONE PERCORSI ---
BASE_PATH = os.path.dirname(__file__)
SILVER_PATH = os.path.join(BASE_PATH, 'silver_layer')
REPORT_PATH = os.path.join(BASE_PATH, 'report_visivi')

def inizializza_ambiente():
    os.makedirs(REPORT_PATH, exist_ok=True)
    sns.set_theme(style="whitegrid")

def carica_dataset():
    path = os.path.join(SILVER_PATH, 'dataset_unificato_pulito.csv')
    if not os.path.exists(path):
        raise FileNotFoundError(f"Dataset non trovato: {path}.")
    return pd.read_csv(path)

def genera_report_frequenze():
    mappa_file = {
        'statistiche_subject culture.csv': 'frequences of answers about user culture',
        'statistiche_evaluation third view video.csv': 'frequences of answers about third view videos',
        'statistiche_evaluation first view image.csv': 'frequences of answers about first view images'
    }

    for csv_file, output_name in mappa_file.items():
        path_csv = os.path.join(SILVER_PATH, csv_file)
        if not os.path.exists(path_csv): continue
            
        df_stats = pd.read_csv(path_csv)
        
        # Grafico Globale
        print(df_stats.head())
         # Debug: verifica tipi colonne
        df_global = df_stats.groupby('Answer_Value').sum(numeric_only=True).reset_index()
        
        df_long_global = pd.melt(df_global, id_vars=['Answer_Value'], var_name='Domanda', value_name='Frequenza')
        print(df_long_global.head())

        plt.figure(figsize=(12, 6))
        sns.barplot(data=df_long_global, x='Domanda', y='Frequenza', hue='Answer_Value', palette='mako')
        plt.title(f"Global distribution: {output_name}")
        plt.xticks(rotation=45)
        plt.savefig(os.path.join(REPORT_PATH, f"{output_name}_GLOBALE.pdf"), bbox_inches='tight')
        plt.close()

        # Grafici IT/DE
        for lingua in ['Italian', 'German']:
            df_lingua = df_stats[df_stats['Language'] == lingua]
            if df_lingua.empty: continue
            df_long_lingua = pd.melt(df_lingua, id_vars=['Answer_Value', 'Language'], var_name='Domanda', value_name='Frequenza')
            plt.figure(figsize=(12, 6))
            sns.barplot(data=df_long_lingua, x='Domanda', y='Frequenza', hue='Answer_Value', palette='viridis')
            plt.title(f"Frequence Distribution ({lingua}): {output_name}")
            plt.xticks(rotation=45)
            plt.savefig(os.path.join(REPORT_PATH, f"{output_name}_{lingua}.pdf"), bbox_inches='tight')
            plt.close()

def esegui_analisi_inferenziale(df):
    df_confronto = df[df['Language'].isin(['Italian', 'German'])].copy()
    if df_confronto['Language'].nunique() < 2: return

    # Definizione gruppi di colonne
    sezioni = {
        "CULTURA": ['Culture_Country', 'Culture_Language', 'Culture_Culture_Self'],
        "ROBOT-HUMAN (VIDEO)": [f'3rd View Video {i+1}' for i in range(6)],
        "ROBOT-YOU (PROSPETTIVA)": [f'1st View Image {i+1}' for i in range(6)]
    }

    path_txt = os.path.join(REPORT_PATH, 'analisi_scientifica_IT_DE.txt')
    with open(path_txt, 'w', encoding='utf-8') as f:
        f.write("====================================================\n")
        f.write("   REPORT STATISTICO DETTAGLIATO: IT vs DE\n")
        f.write("====================================================\n\n")
        
        for nome_sez, cols in sezioni.items():
            f.write(f"--- CATEGORIA: {nome_sez} ---\n")
            
            # 1. Analisi Macro (Media della sezione)
            media_it = df_confronto[df_confronto['Language'] == 'Italian'][cols].mean(axis=1)
            media_de = df_confronto[df_confronto['Language'] == 'German'][cols].mean(axis=1)
            _, p_macro = stats.mannwhitneyu(media_it, media_de)
            
            f.write(f"[ANALISI AGGREGATA]\n")
            f.write(f"  P-Value: {p_macro:.4f} -> {'SIGNIFICATIVO' if p_macro < 0.05 else 'NON SIGNIFICATIVO'}\n\n")
            
            # 2. Analisi Puntuale (Singola Domanda)
            f.write(f"[ANALISI PUNTUALE PER DOMANDA]\n")
            for c in cols:
                val_it = df_confronto[df_confronto['Language'] == 'Italian'][c].dropna()
                val_de = df_confronto[df_confronto['Language'] == 'German'][c].dropna()
                _, p_puntuale = stats.mannwhitneyu(val_it, val_de)
                
                status = "(!)" if p_puntuale < 0.05 else "   "
                f.write(f"  {status} {c:25} | P-Value: {p_puntuale:.4f}\n")
            f.write("\n" + "-"*50 + "\n\n")

    # Boxplot Comparativo
    plt.figure(figsize=(10, 6))
    df_confronto['Media_Totale'] = df_confronto[sezioni["CULTURA"]].mean(axis=1)
    sns.boxplot(data=df_confronto, x='Language', y='Media_Totale', palette='Set2')
    plt.title("Distribution comparison of questions about culture: IT vs DE")
    plt.savefig(os.path.join(REPORT_PATH, 'boxplot_comparativo_culture_IT_DE.pdf'))
    plt.close()

    # Boxplot Comparativo
    plt.figure(figsize=(10, 6))
    df_confronto['Media_Totale'] = df_confronto[sezioni["ROBOT-HUMAN (VIDEO)"]].mean(axis=1)
    sns.boxplot(data=df_confronto, x='Language', y='Media_Totale', palette='Set2')
    plt.title("Distribution comparison of questions about robot-human interaction (third view video): IT vs DE")
    plt.savefig(os.path.join(REPORT_PATH, 'boxplot_comparativo_robot_human_video_IT_DE.pdf'))
    plt.close()

    # Boxplot Comparativo
    plt.figure(figsize=(10, 6))
    df_confronto['Media_Totale'] = df_confronto[sezioni["ROBOT-YOU (PROSPETTIVA)"]].mean(axis=1)
    sns.boxplot(data=df_confronto, x='Language', y='Media_Totale', palette='Set2')
    plt.title("Distribution comparison of questions about robot-human interaction (first view images): IT vs DE")
    plt.savefig(os.path.join(REPORT_PATH, 'boxplot_comparativo_robot_you_perspective_IT_DE.pdf'))
    plt.close()

def main():
    try:
        inizializza_ambiente()
        genera_report_frequenze()
        df = carica_dataset()
        esegui_analisi_inferenziale(df)
        print("Pipeline completata: Analisi per Macro-aree e Singole Domande generata.")
    except Exception as e:
        print(f"Errore: {e}")

if __name__ == "__main__":
    main()