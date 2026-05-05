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
STATS_PATH = os.path.join(BASE_PATH, 'statistiche')

def inizializza_ambiente():
    """Crea le directory necessarie e imposta lo stile dei grafici."""
    os.makedirs(REPORT_PATH, exist_ok=True)
    os.makedirs(STATS_PATH, exist_ok=True)
    sns.set_theme(style="whitegrid")

def cliffs_delta(lst1, lst2):
    """Calcola l'effetto Cliff's Delta per il confronto tra gruppi."""
    m, n = len(lst1), len(lst2)
    diffs = np.array([np.sign(x - y) for x in lst1 for y in lst2])
    return np.mean(diffs)

def get_effetto_label(delta):
    """Classifica l'ampiezza dell'effetto."""
    abs_delta = abs(delta)
    if abs_delta < 0.147: return "Trasc."
    elif abs_delta < 0.33: return "Piccolo"
    elif abs_delta < 0.474: return "Medio"
    else: return "Grande"

def esegui_pipeline_analisi():
    inizializza_ambiente()
    
    # Caricamento del dataset silver completo
    df = pd.read_csv(os.path.join(SILVER_PATH, 'full_silver_dataset.csv'))
    
    # Definizione gruppi di colonne basati sul tuo schema
    sezioni = {
        "Cultura": [col for col in df.columns if "relationship" in col.lower() or "culture" in col.lower() and "nationality" not in col.lower()],
        "Personalità": [col for col in df.columns if "i_see_myself" in col.lower() or "reserved" in col.lower()],
        "Robot_Trust": [col for col in df.columns if "robot" in col.lower()]
    }

    results = []

    for nome_sez, cols in sezioni.items():
        print(f"Elaborazione sezione: {nome_sez}...")
        
        # Filtro per le due nazionalità principali per il confronto[cite: 2, 3]
        df_sub = df[df['Nationality'].isin(['Italian', 'German'])].copy()
        
        for c in cols:
            group_it = df_sub[df_sub['Nationality'] == 'Italian'][c].dropna()
            group_de = df_sub[df_sub['Nationality'] == 'German'][c].dropna()
            
            if len(group_it) < 2 or len(group_de) < 2: continue

            # Analisi Inferenziale (Mann-Whitney U)
            u_stat, p_val = stats.mannwhitneyu(group_it, group_de)
            
            # Dimensione dell'effetto[cite: 3]
            delta = cliffs_delta(group_it, group_de)
            
            results.append({
                'Categoria': nome_sez,
                'Variabile': c,
                'IT_Media': group_it.mean(),
                'DE_Media': group_de.mean(),
                'P_Value': p_val,
                'Cliffs_Delta': delta,
                'Effetto': get_effetto_label(delta)
            })

            # Generazione Grafici (Boxplot)[cite: 2]
            plt.figure(figsize=(8, 5))
            sns.boxplot(data=df_sub, x='Nationality', y=c, palette='Set2')
            plt.title(f"Confronto IT vs DE: {c[:40]}...")
            plt.savefig(os.path.join(REPORT_PATH, f"boxplot_{nome_sez}_{c[:20]}.pdf"))
            plt.close()

    # Salvataggio Statistiche in CSV e LaTeX[cite: 3]
    df_results = pd.DataFrame(results)
    df_results.to_csv(os.path.join(STATS_PATH, 'analisi_differenze_culturali.csv'), index=False)
    
    # Generazione Tabella LaTeX professionale[cite: 3]
    with open(os.path.join(STATS_PATH, 'tabella_risultati.tex'), 'w') as f:
        f.write(df_results.to_latex(index=False, float_format="%.3f", 
                                   caption="Confronto Statistico tra Nazionalità",
                                   label="tab:stat_results"))

if __name__ == "__main__":
    try:
        esegui_pipeline_analisi()
        print("Analisi completata. Controlla le cartelle 'report_visivi' e 'statistiche'.")
    except Exception as e:
        print(f"Errore durante l'analisi: {e}")