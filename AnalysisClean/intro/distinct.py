import pandas as pd
import os

def segregate_introduction_data(input_csv):
    # Create the distinct directory if it doesn't exist
    output_dir = 'distinct'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created directory: {output_dir}")

    # Load the introduction results
    try:
        df = pd.read_csv(input_csv)
    except FileNotFoundError:
        print(f"Error: {input_csv} not found.")
        return

    # Clean any whitespace from Nationality strings
    df['Nationality'] = df['Nationality'].str.strip()

    # 1. Segregate Germans
    german_df = df[df['Nationality'].str.lower() == 'german']
    german_df.to_csv(os.path.join(output_dir, 'german_intro.csv'), index=False)
    
    # 2. Segregate Italians
    italian_df = df[df['Nationality'].str.lower() == 'italian']
    italian_df.to_csv(os.path.join(output_dir, 'italian_intro.csv'), index=False)
    
    # 3. Global Results (The full cleaned dataset)
    df.to_csv(os.path.join(output_dir, 'global_intro.csv'), index=False)

    print(f"Processing complete.")
    print(f"German entries: {len(german_df)}")
    print(f"Italian entries: {len(italian_df)}")
    print(f"Total entries: {len(df)}")

if __name__ == "__main__":
    # Replace with the actual name of your source file
    segregate_introduction_data('./golden_layer/processed_features.csv')