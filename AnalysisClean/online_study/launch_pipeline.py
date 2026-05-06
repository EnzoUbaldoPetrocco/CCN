import bronze2metadata
import bronze2silver
import silver2report
import silver2final_result
import silver2final_result_sep
import silver2tex
import silver2statistics


if __name__ == "__main__":
    print("=== Fase 1: Estrazione e Pulizia Dati (Bronze -> Silver) ===")
    bronze2silver.main()

    print("\n=== Fase 2: Generazione Report Grafici (Silver -> Report) ===")
    bronze2metadata.main()

    print("\n=== Fase 4: Analisi Statistiche Avanzate (Silver -> Statistiche) ===")
    silver2statistics.main()

    print("\n=== Fase 5: Estrazione Distanze Ottimali (Silver -> Final Results) ===")
    silver2final_result.main()

    print("\n=== Fase 5: Estrazione Distanze Ottimali (Silver -> Final Results) ===")
    silver2final_result_sep.main()

    
    print("\n=== Fase 3: Generazione Report Grafici (Silver -> Report) ===")
    silver2report.main()

    print("\n=== Fase 6: Conversione CSV in LaTeX (Silver/Final -> LaTeX) ===")
    silver2tex.main()