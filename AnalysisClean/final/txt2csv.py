import pandas as pd
import re

def clean_text(text):
    """Removes HTML tags and standardizes whitespace."""
    if not isinstance(text, str):
        return text
    # Remove HTML tags
    clean = re.sub(r'<[^>]*>', '', text)
    # Remove excessive whitespace/newlines
    return " ".join(clean.split())

def transform_experimental_data(input_csv, output_csv):
    try:
        # Load the raw CSV
        df = pd.read_csv(input_csv)
        
        # 1. Identify the 'Question' column 
        # Since the header is long and contains HTML, we look for key terms
        target_keywords = ["Neutral", "National Knowledge", "Adaptive"]
        response_col = None
        
        for col in df.columns:
            if all(key in col for key in target_keywords):
                response_col = col
                break
        
        if response_col is None:
            raise ValueError("Could not locate the specific response column in the CSV.")

        # 2. Select and Rename Columns
        # We keep Nationality and Participant ID, and the Response
        cols_to_keep = ['Participant ID', 'Nationality', response_col]
        
        # Verify columns exist (Nationality/ID might have slightly different names)
        # Using a case-insensitive check for robustness
        final_df = df[cols_to_keep].copy()
        final_df = final_df.rename(columns={response_col: "Participant_Response"})

        # 3. Clean Content
        # Apply HTML stripping to the responses
        final_df['Participant_Response'] = final_df['Participant_Response'].apply(clean_text)

        # 4. Add Qualitative Analysis Columns (Theoretical Grounding)
        analysis_columns = [
            "Change_Detected",       # Boolean/Binary
            "Primary_Trigger",       # e.g., Language, Behavior, Style
            "Socio_Cultural_Logic",  # e.g., In-group preference, comfort
            "Grounding_Verification",# Alignment with theoretical intent
            "Researcher_Notes"
        ]
        
        for col in analysis_columns:
            final_df[col] = ""

        # 5. Export to CSV
        final_df.to_csv(output_csv, index=False, encoding='utf-8-sig')
        print(f"Transformation complete. File saved as: {output_csv}")

    except Exception as e:
        print(f"Technical error during transformation: {e}")

# --- EXECUTION ---
INPUT_FILENAME = "risposte_apert.csv" # Change this to your source filename
OUTPUT_FILENAME = "qualitative_analysis_grid.csv"

transform_experimental_data(INPUT_FILENAME, OUTPUT_FILENAME)