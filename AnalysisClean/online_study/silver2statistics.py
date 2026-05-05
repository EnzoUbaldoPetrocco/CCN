import pandas as pd
import numpy as np
import os
from scipy import stats

# --- CONFIGURAZIONE ---
BASE_PATH = os.path.dirname(__file__)
SILVER_PATH = os.path.join(BASE_PATH, 'silver_layer')
REPORT_PATH = os.path.join(BASE_PATH, 'report_visivi')

def normalizza_likert(val):
    """Normalizza scala 1-5 in range 0-1."""
    return (val - 1) / 4

def get_effetto_cliffs_delta(delta):
    """
    Classifica l'effetto secondo Romano et al. (2006).
    Soglie: 0.147 (Small), 0.33 (Medium), 0.474 (Large).
    """
    abs_delta = abs(delta)
    if abs_delta < 0.147:
        return "Trasc."
    elif abs_delta < 0.33:
        return "Piccolo"
    elif abs_delta < 0.474:
        return "Medio"
    else:
        return "Grande"

def genera_latex_table(df_results, output_path):
    """Genera una tabella LaTeX professionale (booktabs + siunitx)."""
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(r"\begin{tabular}{l S[table-format=1.2] S[table-format=1.2] S[table-format=1.2] S[table-format=1.2] S[table-format=1.2] c r}" + "\n")
        f.write(r"\toprule" + "\n")
        f.write(r"& \multicolumn{2}{c}{Media} & \multicolumn{2}{c}{Dev. Std.} & {KS Test} & \multicolumn{2}{c}{Cliff's Delta} \\" + "\n")
        f.write(r"\cmidrule(lr){2-3} \cmidrule(lr){4-5} \cmidrule(lr){7-8}" + "\n")
        f.write(r"Variabile & {IT} & {DE} & {IT} & {DE} & {$p$-value} & {$d$} & {Effetto} \\" + "\n")
        f.write(r"\midrule" + "\n")
        
        last_cat = ""
        for _, row in df_results.iterrows():
            # Aggiunge spazio tra categorie (Culture, Robot_Human, Robot_You)[cite: 1]
            current_cat = row['Variabile'].split('_')[0]
            if last_cat != "" and current_cat != last_cat:
                f.write(r"\addlinespace" + "\n")
            last_cat = current_cat
            
            # Pulizia nome variabile per LaTeX[cite: 1]
            var_name = row['Variabile'].replace('_', r'\_')
            
            f.write(f"{var_name} & {row['IT_Media']:.2f} & {row['DE_Media']:.2f} & "
                    f"{row['IT_Std']:.2f} & {row['DE_Std']:.2f} & {row['KS_p_value']:.2f} & "
                    f"{row['Cliffs_Delta']:.2f} & {row['Effetto']} \\\\\n")
        
        f.write(r"\bottomrule" + "\n")
        f.write(r"\end{tabular}" + "\n")

def calcola_statistiche_avanzate():
    os.makedirs(REPORT_PATH, exist_ok=True)
    df = pd.read_csv(os.path.join(SILVER_PATH, 'dataset_unificato_pulito.csv'))
    
    # Filtro culture di interesse[cite: 1]
    df_study = df[df['Language'].isin(['Italiano', 'Deutsch'])].copy()
    
    likert_cols = [c for c in df_study.columns if 'Culture_' in c or 'Robot_' in c]
    df_study[likert_cols] = df_study[likert_cols].apply(pd.to_numeric, errors='coerce')

    # 1. NORMALIZZAZIONE[cite: 1]
    df_norm = df_study.copy()
    df_norm[likert_cols] = df_norm[likert_cols].apply(normalizza_likert)

    results = []

    for col in likert_cols:
        group_it = df_norm[df_norm['Language'] == 'Italiano'][col].dropna()
        group_de = df_norm[df_norm['Language'] == 'Deutsch'][col].dropna()
        
        if len(group_it) < 2 or len(group_de) < 2: continue

        # Test KS e Cliff's Delta[cite: 1]
        _, ks_p = stats.ks_2samp(group_it, group_de)
        
        def cliffs_delta(lst1, lst2):
            m, n = len(lst1), len(lst2)
            # Calcolo ottimizzato del Delta[cite: 1]
            diffs = np.array([np.sign(x - y) for x in lst1 for y in lst2])
            return np.mean(diffs)

        delta = cliffs_delta(group_it, group_de)

        results.append({
            'Variabile': col,
            'IT_Media': group_it.mean(),
            'IT_Std': group_it.std(),
            'DE_Media': group_de.mean(),
            'DE_Std': group_de.std(),
            'KS_p_value': ks_p,
            'Cliffs_Delta': delta,
            'Effetto': get_effetto_cliffs_delta(delta)
        })

    # Export Risultati[cite: 1]
    df_results = pd.DataFrame(results)
    df_results.to_csv(os.path.join(REPORT_PATH, 'analisi_avanzata_distribuzioni.csv'), index=False)
    
    # Generazione file .tex[cite: 1]
    genera_latex_table(df_results, os.path.join(REPORT_PATH, 'tabella_statistiche.tex'))
    
    print("Elaborazione completata: analisi_avanzata_distribuzioni.csv e tabella_statistiche.tex generati.")

if __name__ == "__main__":
    calcola_statistiche_avanzate()