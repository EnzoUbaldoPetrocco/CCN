import pandas as pd

def extract_descriptive_stats(df_path):
    # Caricamento dati
    df = pd.read_csv(df_path)
    
    # Mapping per chiarezza
    measures = {
        'Culture_Closeness_Avg': 'Cultural Affinity',
        'Competence_Overall': 'Perceived Competence'
    }
    
    # Lista per raccogliere i dati
    stats_list = []

    # 1. Calcolo per Global, Italian e German
    groups = {'Global': df, 
              'Italian': df[df['Nationality'] == 'Italian'], 
              'German': df[df['Nationality'] == 'German']}

    for group_name, data in groups.items():
        # Raggruppiamo per Paradigm (A, B, F)
        summary = data.groupby('Paradigm').agg({
            'Culture_Closeness_Avg': ['mean', 'std', 'var'],
            'Competence_Overall': ['mean', 'std', 'var']
        })
        
        for paradigm in ['A', 'B', 'F']:
            if paradigm in summary.index:
                stats_list.append({
                    'Cohort': group_name,
                    'Paradigm': paradigm,
                    'Affinity_Mean': summary.loc[paradigm, ('Culture_Closeness_Avg', 'mean')],
                    'Affinity_Std': summary.loc[paradigm, ('Culture_Closeness_Avg', 'std')],
                    'Comp_Mean': summary.loc[paradigm, ('Competence_Overall', 'mean')],
                    'Comp_Std': summary.loc[paradigm, ('Competence_Overall', 'std')]
                })

    # Creazione DataFrame finale
    res_df = pd.DataFrame(stats_list)

    # Formattazione per LaTeX: Media ± SD
    res_df['Cultural Affinity ($\mu \pm \sigma$)'] = res_df.apply(
        lambda x: f"{x['Affinity_Mean']:.3f} \pm {x['Affinity_Std']:.3f}", axis=1)
    res_df['Competence ($\mu \pm \sigma$)'] = res_df.apply(
        lambda x: f"{x['Comp_Mean']:.3f} \pm {x['Comp_Std']:.3f}", axis=1)

    # Selezione colonne finali
    final_table = res_df[['Cohort', 'Paradigm', 'Cultural Affinity ($\mu \pm \sigma$)', 'Competence ($\mu \pm \sigma$)']]

    print("\n--- TABELLA DESCRITTIVA LATEX ---\n")
    print(final_table.to_latex(index=False, escape=False, column_format='llcc'))

# Esegui puntando al tuo CSV
extract_descriptive_stats('golden_layer/interaction_features_full.csv')