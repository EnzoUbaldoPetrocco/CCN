import pandas as pd
import os
from textblob import TextBlob

# --- CONFIGURATION ---
BASE_PATH = os.path.dirname(__file__)
INPUT_PATH = os.path.join(BASE_PATH, 'bronze_layer', 'proxemics_raw.csv') 
OUTPUT_DIR = os.path.join(BASE_PATH, 'silver_layer')

def get_sentiment(text):
    if pd.isna(text) or str(text).lower() == 'nan': return 0
    return TextBlob(str(text)).sentiment.polarity

def process_proxemics():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    df = pd.read_csv(INPUT_PATH)
    
    # Standardize columns
    df = df.loc[:, ~df.columns.str.contains('^Unnamed')]
    
    # Process Categorical Proxemics
    categories = ['Head orientation', 'Torso orientation', 'Arms/Hands', 'Facial affect', 'Proxemics']
    df_clean = pd.DataFrame()
    df_clean['Paradigm'] = df['Paradigm']
    # Add Nationality if present in your source
    if 'Nationality' in df.columns: df_clean['Nationality'] = df['Nationality']

    for cat in categories:
        opt_col = f"{cat} (optionals)" if f"{cat} (optionals)" in df.columns else f"{cat} (optional)"
        df_clean[cat] = df[cat].astype(str) + ", " + df[opt_col].fillna("").astype(str)
        df_clean[cat] = df_clean[cat].str.strip(", ")

    # Process "txt" (Open Questions)
    # Assuming column name is 'Open_Comments' - change to match your header
    text_col = [c for c in df.columns if '?' in c or 'Comments' in c][0] 
    df_clean['Sentiment_Score'] = df[text_col].apply(get_sentiment)

    df_clean.to_csv(os.path.join(OUTPUT_DIR, 'proxemics_silver.csv'), index=False)

if __name__ == "__main__":
    process_proxemics()