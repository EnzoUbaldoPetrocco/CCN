import pandas as pd
import os

# Define paths
input_path = './silver_layer/intro_study_silver.csv'
output_folder = 'golden_layer'
output_path = os.path.join(output_folder, 'processed_features.csv')

# Ensure output directory exists
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

def process_data():
    # Load the silver layer data
    df = pd.read_csv(input_path)

    # 1. Extract 14-item Trust Perception (Mean of Robot_Trust_Q1 to Q14)
    trust_cols = [f'Robot_Trust_Q{i}' for i in range(1, 15)]
    df['Trust_Overall_Score'] = df[trust_cols].mean(axis=1) / 100

    # 2. Extract OCEAN Personality Traits (BFI-10 mapping)
    # Mapping based on standard BFI-10 sub-scales:
    # Extraversion (Q1, Q6), Agreeableness (Q2, Q7), Conscientiousness (Q3, Q8), 
    # Neuroticism (Q4, Q9), Openness (Q5, Q10)
    df['Personality_Extraversion'] = (df[['Personality_BFI_Q1', 'Personality_BFI_Q6']].mean(axis=1) - 1) / 4
    df['Personality_Agreeableness'] = (df[['Personality_BFI_Q2', 'Personality_BFI_Q7']].mean(axis=1) - 1) / 4
    df['Personality_Conscientiousness'] = (df[['Personality_BFI_Q3', 'Personality_BFI_Q8']].mean(axis=1) - 1) / 4
    df['Personality_Neuroticism'] = (df[['Personality_BFI_Q4', 'Personality_BFI_Q9']].mean(axis=1) - 1) / 4
    df['Personality_Openness'] = (df[['Personality_BFI_Q5', 'Personality_BFI_Q10']].mean(axis=1) - 1) / 4

    # 3. Extract Cultural Affinity Average
    culture_cols = [f'Culture_Affinity_Q{i}' for i in range(1, 4)]
    df['Cultural_Affinity_Score'] = (df[culture_cols].mean(axis=1) - 1) / 6

    # Select only high-level features for the Golden Layer
    golden_df = df[[
        'Participant_ID', 
        'Nationality', 
        'Trust_Overall_Score',
        'Personality_Extraversion',
        'Personality_Agreeableness',
        'Personality_Conscientiousness',
        'Personality_Neuroticism',
        'Personality_Openness',
        'Cultural_Affinity_Score'
    ]]

    # Export to golden_layer
    golden_df.to_csv(output_path, index=False)
    print(f"Transformation complete. File saved to {output_path}")

if __name__ == "__main__":
    process_data()