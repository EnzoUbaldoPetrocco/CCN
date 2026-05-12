import pandas as pd
import io

def process_questionnaire_stats():
    # Read and clean data
    # Path remains as per your silver layer structure
    df = pd.read_csv("./silver_layer/intro_study_silver.csv")
    df['Nationality'] = df['Nationality'].str.strip()
    
    # Identify numeric columns (skipping ID)
    numeric_cols = df.select_dtypes(include=['number']).columns.tolist()

    # Apply Hardcoded Normalization [0,1]
    for col in numeric_cols:
        if 'Trust' in col:
            # Scale 0-100
            df[col] = df[col] / 100.0
        elif 'Affinity' in col:
            # Scale 1-7 (Affinity questions)
            df[col] = (df[col] - 1) / 6.0
        else:
            # Scale 1-5 (BFI Personality questions)
            df[col] = (df[col] - 1) / 4.0
    
    # Separate cohorts and calculate statistics
    german_stats = df[df['Nationality'] == 'German'][numeric_cols].agg(['mean', 'std']).T
    italian_stats = df[df['Nationality'] == 'Italian'][numeric_cols].agg(['mean', 'std']).T
    
    # Output Consolidated LaTeX Table
    print(r"% --- Comparative Cohort Statistics ---")
    print(r"\begin{table}[ht]")
    print(r"\centering")
    print(r"\caption{Comparative Descriptive Statistics: German and Italian Questionnaire Responses}")
    print(r"\begin{tabular}{lcc}")
    print(r"\toprule")
    print(r"Question & German ($\mu \pm \sigma$) & Italian ($\mu \pm \sigma$) \\")
    print(r"\midrule")
    
    for idx in numeric_cols:
        q_name = idx.replace('_', ' ')
        
        # Extract German values
        g_m = german_stats.loc[idx, 'mean']
        g_s = german_stats.loc[idx, 'std']
        
        # Extract Italian values
        i_m = italian_stats.loc[idx, 'mean']
        i_s = italian_stats.loc[idx, 'std']
        
        # Print row with both values
        print(f"{q_name} & ${g_m:.3f} \pm {g_s:.3f}$ & ${i_m:.3f} \pm {i_s:.3f}$ \\\\")
        
    print(r"\bottomrule")
    print(r"\end{tabular}")
    print(r"\end{table}")

    german_stats = df[df['Nationality'] == 'German'][numeric_cols].to_csv("./distinct/german_low_intro.csv", index=False)
    italian_stats = df[df['Nationality'] == 'Italian'][numeric_cols].to_csv("./distinct/italian_low_intro.csv", index=False)
    global_stats = df[numeric_cols].to_csv("./distinct/global_low_intro.csv", index=False)

if __name__ == "__main__":
    process_questionnaire_stats()