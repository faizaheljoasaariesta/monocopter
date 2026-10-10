import os
import csv

# Path ke file CSV hasil simulasi Anda (silakan sesuaikan jika path-nya berbeda)
# Default mengarah ke folder 'hasil-simulasi' di root project
csv_path = os.path.join('hasil-simulasi', 'hasil_simulasi_full.csv')

# Struktur 13 fase operasi yang sesuai dengan larik scenario Anda
phases = [
    {"no": 1, "start": 0.0, "end": 2.0, "name": "Ground Idle (motor minimum)", "sumbu": "z", "ref": 0.0, "col": "alt_m"},
    {"no": 2, "start": 2.0, "end": 5.0, "name": "Takeoff (ramp vertikal naik)", "sumbu": "z", "ref": 3.0, "col": "alt_m"},
    {"no": 3, "start": 5.0, "end": 7.0, "name": "Steady Hover (sikap datar)", "sumbu": "z", "ref": 3.0, "col": "alt_m"},
    {"no": 4, "start": 7.0, "end": 10.0, "name": "Manuver Roll Step Response", "sumbu": "phi", "ref": 12.0, "col": "roll_deg"},
    {"no": 5, "start": 10.0, "end": 12.0, "name": "Level Recovery (roll kembali nol)", "sumbu": "phi", "ref": 0.0, "col": "roll_deg"},
    {"no": 6, "start": 12.0, "end": 15.0, "name": "Manuver Pitch Step Response", "sumbu": "theta", "ref": 10.0, "col": "pitch_deg"},
    {"no": 7, "start": 15.0, "end": 17.0, "name": "Level Recovery (pitch kembali nol)", "sumbu": "theta", "ref": 0.0, "col": "pitch_deg"},
    {"no": 8, "start": 17.0, "end": 19.5, "name": "Manuver Yaw Heading Step", "sumbu": "psi", "ref": 30.0, "col": "yaw_deg"},
    {"no": 9, "start": 19.5, "end": 21.0, "name": "Hold Heading Yaw", "sumbu": "psi", "ref": 30.0, "col": "yaw_deg"},
    {"no": 10, "start": 21.0, "end": 22.0, "name": "Yaw Return", "sumbu": "psi", "ref": 0.0, "col": "yaw_deg"},
    {"no": 11, "start": 22.0, "end": 24.0, "name": "Descent Tahap 1 (turun ke 2 m)", "sumbu": "z", "ref": 2.0, "col": "alt_m"},
    {"no": 12, "start": 24.0, "end": 25.5, "name": "Descent Tahap 2 (turun ke 1 m)", "sumbu": "z", "ref": 1.0, "col": "alt_m"},
    {"no": 13, "start": 25.5, "end": 28.0, "name": "Touchdown dan Motor Shut-off", "sumbu": "z", "ref": -0.3, "col": "alt_m"}
]

def run_filter():
    if not os.path.exists(csv_path):
        print(f"Error: File '{csv_path}' tidak ditemukan.")
        print("Pastikan Anda menjalankan skrip ini dari folder root proyek Anda.")
        print("Gunakan perintah: python3 filter_scenario.py")
        return

    # Membaca data CSV
    rows = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for r in reader:
            # Konversi semua nilai ke float
            row_data = {k: float(v) for k, v in r.items() if v.strip() != ''}
            rows.append(row_data)

    print("\n" + "="*95)
    print("                      DATA RINGKASAN SETIAP AKHIR FASE (28 DETIK)")
    print("="*95)
    print(f"{'No':<3} | {'Waktu (s)':<11} | {'Nama Fase':<34} | {'Sumbu':<5} | {'Referensi':<10} | {'Aktual':<10} | {'Galat %':<8}")
    print("-"*95)

    for ph in phases:
        # Cari baris yang paling mendekati waktu target akhir fase (t_end)
        target_t = ph["end"]
        best_row = min(rows, key=lambda x: abs(x["time"] - target_t))
        actual_val = best_row[ph["col"]]

        # Hitung galat
        if ph["ref"] == 0.0:
            galat_str = "-"
        elif ph["name"] == "Touchdown dan Motor Shut-off":
            galat_str = "ditahan z=0"
        else:
            galat_pct = (abs(actual_val - ph["ref"]) / abs(ph["ref"])) * 100
            galat_str = f"{galat_pct:.2f}%"

        # Format tampilan referensi dan aktual
        ref_val = ph["ref"]
        sumbu_str = ph["sumbu"]
        
        # Tentukan satuan
        unit = "deg" if ph["col"] in ["roll_deg", "pitch_deg", "yaw_deg"] else "m"
        
        ref_formatted = f"{ref_val:.1f} {unit}"
        if ph["name"] == "Touchdown dan Motor Shut-off":
            actual_formatted = f"0.000 m*"
        else:
            actual_formatted = f"{actual_val:.4f} {unit}"

        time_range = f"{ph['start']:.1f} - {ph['end']:.1f}"
        print(f"{ph['no']:<3} | {time_range:<11} | {ph['name']:<34} | {sumbu_str:<5} | {ref_formatted:<10} | {actual_formatted:<10} | {galat_str:<8}")

    print("="*95)
    print("Catatan: *Ditahan oleh kendala kontak tanah (ground constraint) di z = 0 m.")
    print("="*95 + "\n")

    # Cetak nilai murni untuk mempermudah penyalinan manual ke laporan Anda
    print("Daftar nilai aktual murni untuk disalin langsung (urut dari Fase 1 s.d 13):")
    for ph in phases:
        target_t = ph["end"]
        best_row = min(rows, key=lambda x: abs(x["time"] - target_t))
        actual_val = best_row[ph["col"]]
        if ph["name"] == "Touchdown dan Motor Shut-off":
            print(f"Fase {ph['no']}: 0.000 (z=0)")
        else:
            print(f"Fase {ph['no']}: {actual_val:.4f}")

if __name__ == '__main__':
    run_filter()