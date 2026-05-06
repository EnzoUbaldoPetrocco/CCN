import pandas as pd
import numpy as np
import os

# --- CONFIGURATION ---
BASE_PATH = os.path.dirname(__file__)
INPUT_PATH = os.path.join(BASE_PATH, 'bronze_layer', 'center.csv') 
OUTPUT_DIR = os.path.join(BASE_PATH, 'silver_layer')

def initialize():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

def process_experiment2_data():
    if not os.path.exists(INPUT_PATH):
        print(f"Error: {INPUT_PATH} not found.")
        return

    df_raw = pd.read_csv(INPUT_PATH)
    
    # Filter for target nationalities
    target_nationalities = ['Italian', 'German']
    df = df_raw[df_raw['Nationality'].isin(target_nationalities)].copy()

    # Column Mapping based on your header
    # P = Paradigm, Nationality = Culture
    culture_closeness_cols = df.columns[4:7]
    competence_cols = df.columns[7:13]

    df_clean = pd.DataFrame()
    df_clean['Paradigm'] = df['P']
    df_clean['Nationality'] = df['Nationality']
    df_clean['Participant_ID'] = df['Participant ID']

    # Transform Culture Closeness (Likert 1-7)
    for i, col in enumerate(culture_closeness_cols):
        df_clean[f'Culture_Closeness_Q{i+1}'] = pd.to_numeric(df[col], errors='coerce')

    # Transform Competence (Scale 1-9)
    for i, col in enumerate(competence_cols):
        df_clean[f'Competence_Q{i+1}'] = pd.to_numeric(df[col], errors='coerce')

    # Export Silver Dataset
    output_path = os.path.join(OUTPUT_DIR, 'experiment2_silver.csv')
    df_clean.to_csv(output_path, index=False)
    print(f"Success: {len(df_clean)} records processed and saved to {output_path}.")

if __name__ == "__main__":
    initialize()
    process_experiment2_data()