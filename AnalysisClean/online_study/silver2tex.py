import pandas as pd
import os
import glob

def convert_csv_directory_to_latex(input_dir: str, output_subdir: str = 'latex_reports'):
    """
    Scansiona una directory alla ricerca di file CSV e genera report LaTeX 
    per ogni tabella trovata, mantenendo l'integrità dei dati numerici.
    """
    # Verifica esistenza directory di input
    if not os.path.exists(input_dir):
        print(f"Errore: La directory '{input_dir}' non esiste.")
        return

    # Percorso per i report LaTeX
    latex_path = os.path.join(input_dir, output_subdir)
    os.makedirs(latex_path, exist_ok=True)

    # Ricerca ricorsiva di tutti i file .csv nella directory
    csv_files = glob.glob(os.path.join(input_dir, "**/*.csv"), recursive=True)

    if not csv_files:
        print(f"Nessun file CSV trovato in {input_dir}.")
        return

    for csv_file in csv_files:
        # Evita di processare file già presenti nella cartella dei report
        if output_subdir in csv_file:
            continue

        try:
            # Caricamento dati
            df = pd.read_csv(csv_file)
            
            # Generazione nome file .tex basato sul file originale
            base_name = os.path.basename(csv_file).replace('.csv', '.tex')
            tex_file_path = os.path.join(latex_path, base_name)

            # Formattazione professionale del nome tabella (Caption)
            caption_name = base_name.replace('.tex', '').replace('_', ' ').title()

            # Conversione in LaTeX
            # Nota: round(3) per precisione scientifica, escape=True per gestire caratteri speciali
            latex_table = df.round(3).to_latex(
                index=False,
                caption=f"Tabella estratta da: {caption_name}",
                label=f"tab:{base_name.replace('.tex', '')}",
                column_format='l' + 'c' * (len(df.columns) - 1),
                escape=True,
                position='htbp',
                longtable=False
            )

            # Scrittura su disco
            with open(tex_file_path, 'w', encoding='utf-8') as f:
                f.write(latex_table)

            print(f"Convertito: {os.path.basename(csv_file)} -> {base_name}")

        except Exception as e:
            print(f"Errore nella conversione di {csv_file}: {e}")

def main():
    # Specifica la cartella dove si trovano i tuoi CSV Silver
    SILVER_DIRECTORY = "silver_layer"
    convert_csv_directory_to_latex(SILVER_DIRECTORY)
    SILVER_DIRECTORY = "final_results_sep"
    convert_csv_directory_to_latex(SILVER_DIRECTORY)
    SILVER_DIRECTORY = "report_finali"
    convert_csv_directory_to_latex(SILVER_DIRECTORY)
    SILVER_DIRECTORY = "report_visivi"
    convert_csv_directory_to_latex(SILVER_DIRECTORY)


if __name__ == "__main__":
    main()