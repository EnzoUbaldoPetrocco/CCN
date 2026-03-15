import pandas as pd

# 1. Load your CSV files
df_main = pd.read_csv('annotations.csv')
df_lookup = pd.read_csv('Video2Paradigm.csv')

# 2. Merge the dataframes
# left_on/right_on tells pandas that 'ID Utente' is the same as 'ID', etc.
df_combined = pd.merge(
    df_main, 
    df_lookup, 
    left_on=['ID Utente', 'Nome del video'], 
    right_on=['ID', 'Video'],
    how='left'
)

# 3. Create the final version
# We drop the old identifiers and the redundant ID/Video columns from the second file
cols_to_drop = ['ID Utente', 'Nome del video', 'ID', 'Video', 'Informazioni cronologiche', 'Io sono']
df_final = df_combined.drop(columns=cols_to_drop)

# 4. Reorder to put 'Paradigm' at the front (optional but cleaner)
cols = ['Paradigm'] + [c for c in df_final.columns if c != 'Paradigm']
df_final = df_final[cols]

# 5. Save the result
df_final.to_csv('bronze.csv', index=False)

print("File created successfully with 'Paradigm' column.")