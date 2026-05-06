import pandas as pd
import numpy as np
import os

# --- CONFIGURATION ---
BASE_PATH = os.path.dirname(__file__)
INPUT_PATH = os.path.join(BASE_PATH, 'bronze_layer', 'Intro CCN  (Risposte).CSV') 
OUTPUT_DIR = os.path.join(BASE_PATH, 'silver_layer')

def initialize():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

def process_intro_data():
    # 1. LOAD DATA
    # Ensure correct encoding if the CSV contains special characters
    df_raw = pd.read_csv(INPUT_PATH)
    
    # 2. FILTER NATIONALITY & CLEAN
    # Based on your snippet: "Italian" and "German" are the values
    target_nationalities = ['Italian', 'German']
    df = df_raw[df_raw['Nationality'].isin(target_nationalities)].copy()

    # 3. MAPPING DICTIONARIES (Likert 1-5)
    personality_likert = {
        'Disagree strongly': 1,
        'Disagree a little': 2,
        'Neither agree or disagree': 3,
        'Agree a little': 4,
        'Agree strongly': 5
    }

    # 4. COLUMN IDENTIFICATION
    # Culture (Indices 3, 4, 5 based on your snippet)
    culture_source = df.columns[3:6]
    
    # Personality
    personality_source = [col for col in df.columns if "I see myself as someone who" in col]
    
    # Trust
    trust_source = [col for col in df.columns if "Consider expectations toward Pepper" in col]

    # 5. TRANSFORMATION & RENAMING
    df_clean = pd.DataFrame()
    df_clean['Nationality'] = df['Nationality']
    df_clean['Participant_ID'] = df['Participant ID']

    # Culture (Likert 1-7)
    for i, col in enumerate(culture_source):
        df_clean[f'Culture_Affinity_Q{i+1}'] = pd.to_numeric(df[col], errors='coerce')

    # Personality (Likert 1-5)
    for i, col in enumerate(personality_source):
        df_clean[f'Personality_BFI_Q{i+1}'] = df[col].map(personality_likert)

    # Trust (0-100%) - Strip '%' and convert to float
    for i, col in enumerate(trust_source):
        val = df[col].astype(str).str.replace('%', '')
        df_clean[f'Robot_Trust_Q{i+1}'] = pd.to_numeric(val, errors='coerce')

    # 6. EXPORT FULL SILVER DATASET
    full_silver_path = os.path.join(OUTPUT_DIR, 'intro_study_silver.csv')
    df_clean.to_csv(full_silver_path, index=False)

    # 7. GENERATE FREQUENCY STATISTICS (For the separated charts)
    # This part was missing and caused the "void" or missing charts
    generate_frequencies(df_clean)

    print(f"Success: {len(df_clean)} participants processed.")
    print(f"Files saved in: {OUTPUT_DIR}")

def generate_frequencies(df):
    """
    Creates the 'statistiche_...' CSVs needed for 
    side-by-side frequency distributions.
    """
    topics = {
        'culture': [c for c in df.columns if 'Culture_Affinity' in c],
        'personality': [c for c in df.columns if 'Personality_BFI' in c],
        'trust': [c for c in df.columns if 'Robot_Trust' in c]
    }

    for topic_name, columns in topics.items():
        all_freqs = []
        for col in columns:
            # Group by nationality and count each answer value
            counts = df.groupby(['Nationality', col]).size().unstack(fill_value=0)
            
            # Ensure all values 1-7 (Culture) or 1-5 (Personality) exist in the table
            max_val = 7 if topic_name == 'culture' else 5
            for val in range(1, max_val + 1):
                if val not in counts.columns:
                    counts[val] = 0
            
            # Format to match your previous silver2report expectation
            counts = counts.sort_index(axis=1).reset_index()
            counts['Variable'] = col
            all_freqs.append(counts)

        # Save frequency file
        pd.concat(all_freqs).to_csv(
            os.path.join(OUTPUT_DIR, f'statistiche_intro_{topic_name}.csv'), 
            index=False
        )

if __name__ == "__main__":
    initialize()
    process_intro_data()