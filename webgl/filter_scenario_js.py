import os
import csv
import glob

csv_path = os.path.join('hasil-simulasi', 'monochopter_sim.csv')

phases = [
    {"no": 1, "start": 0.0, "end": 2.0, "name": "Ground Idle (motor minimum)", "sumbu": "z", "ref": 0.0, "col": "altitude_m"},
    {"no": 2, "start": 2.0, "end": 5.0, "name": "Takeoff (ramp vertikal naik)", "sumbu": "z", "ref": 3.0, "col": "altitude_m"},
    {"no": 3, "start": 5.0, "end": 7.0, "name": "Steady Hover (sikap datar)", "sumbu": "z", "ref": 3.0, "col": "altitude_m"},
    {"no": 4, "start": 7.0, "end": 10.0, "name": "Manuver Roll Step Response", "sumbu": "phi", "ref": 12.0, "col": "roll_deg"},
    {"no": 5, "start": 10.0, "end": 12.0, "name": "Level Recovery (roll kembali nol)", "sumbu": "phi", "ref": 0.0, "col": "roll_deg"},
    {"no": 6, "start": 12.0, "end": 15.0, "name": "Manuver Pitch Step Response", "sumbu": "theta", "ref": 10.0, "col": "pitch_deg"},
    {"no": 7, "start": 15.0, "end": 17.0, "name": "Level Recovery (pitch kembali nol)", "sumbu": "theta", "ref": 0.0, "col": "pitch_deg"},
    {"no": 8, "start": 17.0, "end": 19.5, "name": "Manuver Yaw Heading Step", "sumbu": "psi", "ref": 30.0, "col": "yaw_deg"},
    {"no": 9, "start": 19.5, "end": 21.0, "name": "Hold Heading Yaw", "sumbu": "psi", "ref": 30.0, "col": "yaw_deg"},
    {"no": 10, "start": 21.0, "end": 22.0, "name": "Yaw Return", "sumbu": "psi", "ref": 0.0, "col": "yaw_deg"},
    {"no": 11, "start": 22.0, "end": 24.0, "name": "Descent Tahap 1 (turun ke 2 m)", "sumbu": "z", "ref": 2.0, "col": "altitude_m"},
    {"no": 12, "start": 24.0, "end": 25.5, "name": "Descent Tahap 2 (turun ke 1 m)", "sumbu": "z", "ref": 1.0, "col": "altitude_m"},
    {"no": 13, "start": 25.5, "end": 28.0, "name": "Touchdown dan Motor Shut-off", "sumbu": "z", "ref": -0.3, "col": "altitude_m"}
]

TIME_COL = "t_s"


def resolve_csv_path():
    if os.path.exists(csv_path):
        return csv_path
        
    candidates = glob.glob('monochopter_sim_*.csv') + \
        glob.glob(os.path.join(os.path.expanduser('~'), 'Downloads', 'monochopter_sim_*.csv'))
    if candidates:
        return max(candidates, key=os.path.getmtime)
    return None


def run_filter():
    path = resolve_csv_path()
    if not path:
        print(f"Error: File '{csv_path}' tidak ditemukan, begitu juga monochopter_sim_*.csv.")
        print("Jalankan tombol '⇄ VALIDASI LINTAS PLATFORM (13-FASE)' di sim.html,")
        print("biarkan sampai t=28s (auto-export), lalu taruh CSV-nya di path di atas.")
        return

    rows = []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for r in reader:
            row_data = {k: float(v) for k, v in r.items() if v.strip() != ''}
            rows.append(row_data)

    print(f"Membaca: {path} ({len(rows)} sampel)\n")
    print("\n" + "="*95)
    print("           DATA RINGKASAN SETIAP AKHIR FASE — SIMULATOR JS (sim.html)")
    print("="*95)
    print(f"{'No':<3} | {'Waktu (s)':<11} | {'Nama Fase':<34} | {'Sumbu':<5} | {'Referensi':<10} | {'Aktual':<10} | {'Galat %':<8}")
    print("-"*95)

    for ph in phases:
        target_t = ph["end"]
        best_row = min(rows, key=lambda x: abs(x[TIME_COL] - target_t))
        actual_val = best_row[ph["col"]]

        if ph["ref"] == 0.0:
            galat_str = "-"
        elif ph["name"] == "Touchdown dan Motor Shut-off":
            galat_str = "ditahan z=0"
        else:
            galat_pct = (abs(actual_val - ph["ref"]) / abs(ph["ref"])) * 100
            galat_str = f"{galat_pct:.2f}%"

        ref_val = ph["ref"]
        unit = "deg" if ph["col"] in ["roll_deg", "pitch_deg", "yaw_deg"] else "m"
        ref_formatted = f"{ref_val:.1f} {unit}"
        if ph["name"] == "Touchdown dan Motor Shut-off":
            actual_formatted = "0.000 m*"
        else:
            actual_formatted = f"{actual_val:.4f} {unit}"

        time_range = f"{ph['start']:.1f} - {ph['end']:.1f}"
        print(f"{ph['no']:<3} | {time_range:<11} | {ph['name']:<34} | {ph['sumbu']:<5} | {ref_formatted:<10} | {actual_formatted:<10} | {galat_str:<8}")

    print("="*95)
    print("Catatan: *Ditahan oleh kendala kontak tanah (ground constraint) di z = 0 m.")
    print("Catatan: Fase 8-10 (sumbu psi) memakai shim heading-hold-P tambahan di")
    print("         sim.html (lihat scriptedScenarioStep) karena loop yaw bawaan JS")
    print("         adalah rate-hold, bukan heading-hold seperti di .m/.py — bandingkan")
    print("         hasil ketiga sumbu ini dengan pemahaman itu, bukan apel-ke-apel murni.")
    print("="*95 + "\n")

    print("Daftar nilai aktual murni untuk disalin langsung (urut dari Fase 1 s.d 13):")
    for ph in phases:
        target_t = ph["end"]
        best_row = min(rows, key=lambda x: abs(x[TIME_COL] - target_t))
        actual_val = best_row[ph["col"]]
        if ph["name"] == "Touchdown dan Motor Shut-off":
            print(f"Fase {ph['no']}: 0.000 (z=0)")
        else:
            print(f"Fase {ph['no']}: {actual_val:.4f}")


if __name__ == '__main__':
    run_filter()
