import re
import pandas as pd
import numpy as np
from scipy import stats
import matplotlib.pyplot as plt
import seaborn as sns
import os

# --- CONFIGURATION ---
CSV_PATH = r"C:\Users\Utente\Desktop\CCN\DataAnalysis\online_study\online_study.csv"
OUTPUT_DIR = r"C:\Users\Utente\Desktop\CCN\DataAnalysis\online_study\results"
os.makedirs(OUTPUT_DIR, exist_ok=True)

def load_and_segment_data(path):
    # Using low_memory=False to handle the mixed-type columns common in survey exports
    df_raw = pd.read_csv(path, low_memory=False)
    
    # Detect language column (flexible naming)
    lang_col = next((c for c in df_raw.columns if "Language" in c or "Lingua" in c), None)
    if not lang_col:
        raise ValueError("Could not find the Language column in the CSV.")
    
    # Filter for target groups only
    df = df_raw[df_raw[lang_col].str.contains('Italiano|Deutsch', na=False)].copy()
    
    # Identify survey columns by keyword
    it_cols = [c for c in df.columns if re.search(r"Quale immagine|Valuta", c, re.I)]
    de_cols = [c for c in df.columns if re.search(r"Welches Bild|Bewerten", c, re.I)]
    
    # Ensure we align the first 15 questions
    it_cols = it_cols[:15]
    de_cols = de_cols[:15]
    
    processed_data = []
    
    for _, row in df.iterrows():
        lang = "Italiano" if "Italiano" in str(row[lang_col]) else "Deutsch"
        cols = it_cols if lang == 'Italiano' else de_cols
        
        # Convert values to numeric, forcing errors to NaN
        vals = pd.to_numeric(row[cols], errors='coerce').values
        
        # Mapping your 3-6-6 structure
        for i, v in enumerate(vals):
            if pd.isna(v): continue
            
            if i < 3:
                cat = "1_Cultural_Closeness"
                label = f"Culture_Q{i+1}"
            elif 3 <= i < 9:
                cat = "2_Third_Person_Proxemics"
                label = f"GIF_Q{i-2}"
            else:
                cat = "3_First_Person_Matrix"
                label = f"Matrix_Q{i-8}"
                
            processed_data.append({
                "Language": lang,
                "Category": cat,
                "Question_Label": label,
                "Rating": v
            })
                
    return pd.DataFrame(processed_data)

def run_analysis():
    data = load_and_segment_data(CSV_PATH)
    
    sns.set_style("whitegrid")
    categories = sorted(data['Category'].unique())
    
    summary_results = []

    for cat in categories:
        cat_data = data[data['Category'] == cat]
        
        # 1. Create the Plot for this specific category
        plt.figure(figsize=(10, 6))
        
        # Using a Violin plot + Strip plot to see the actual density and individual points
        sns.violinplot(data=cat_data, x="Question_Label", y="Rating", hue="Language", 
                       split=True, inner="quartile", palette="muted")
        
        plt.title(f"Distribution Comparison: {cat.replace('_', ' ')}")
        plt.ylabel("Rating Score")
        plt.xlabel("Question")
        plt.legend(title="Language", loc='upper right')
        
        # Save each category as a separate file
        file_name = f"distribution_{cat}.png"
        plt.tight_layout()
        plt.savefig(os.path.join(OUTPUT_DIR, file_name), dpi=300)
        print(f"Saved: {file_name}")
        plt.close()

        # 2. Statistical Comparison (Mann-Whitney U)
        for q in cat_data['Question_Label'].unique():
            q_subset = cat_data[cat_data['Question_Label'] == q]
            it_vals = q_subset[q_subset['Language'] == 'Italiano']['Rating']
            de_vals = q_subset[q_subset['Language'] == 'Deutsch']['Rating']
            
            if len(it_vals) > 1 and len(de_vals) > 1:
                u_stat, p_val = stats.mannwhitneyu(it_vals, de_vals)
                summary_results.append({
                    "Category": cat,
                    "Question": q,
                    "IT_Mean": round(it_vals.mean(), 2),
                    "DE_Mean": round(de_vals.mean(), 2),
                    "MW_U_Stat": u_stat,
                    "p_value": round(p_val, 4),
                    "Significant": "Yes" if p_val < 0.05 else "No"
                })

    # Save stats to a single CSV
    stats_df = pd.DataFrame(summary_results)
    stats_df.to_csv(os.path.join(OUTPUT_DIR, "statistical_comparison.csv"), index=False)
    print("\nStatistical summary saved to 'statistical_comparison.csv'")

if __name__ == "__main__":
    run_analysis()