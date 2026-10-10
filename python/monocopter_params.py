"""
================================================================
FILE: monochopter_params.py (v4 - PERBAIKAN BENTUK PID)
TUJUAN: Parameter simulasi monochopter (versi FINAL v4)
================================================================
KONSISTEN dengan monochopter_params_v4.m (MATLAB).

================================================================
RIWAYAT PERBAIKAN v3 -> v4:

  Ditemukan bahwa skrip validasi (step_response_*.py) dan
  kemungkinan besar blok PID Simulink memakai bentuk PID PADA
  GALAT (u = Kp*e + Ki*int(e) + Kd*de/dt), padahal Bab II
  (Persamaan hukum_kendali_state_feedback) menurunkan dan Bab III
  memakai bentuk UMPAN BALIK KEADAAN:

      u = -Kp*y - Kd*dy + Ki*xI          <-- BENAR, dipakai desain

  BUKAN bentuk galat standar:

      u = Kp*e + Ki*int(e) + Kd*de/dt    <-- SALAH, jangan dipakai

  Kedua bentuk memiliki KUTUB lingkar tertutup identik (sama-sama
  hasil pole placement), tapi ZERO fungsi alihnya berbeda,
  sehingga bentuk PID-pada-galat menghasilkan overshoot yang jauh
  lebih besar dari prediksi zeta yang dirancang (dibuktikan lewat
  simulasi: 0,6% seharusnya menjadi ~15-21% teramati).

  Konsekuensi kedua: batas saturasi integrator (int_*_limit di
  Bagian 8-11, dan pid_*_int_max di Bagian 11b) yang lama HANYA
  valid untuk bentuk PID-pada-galat (yang di steady-state butuh
  xI->0). Bentuk umpan-balik-keadaan justru butuh xI menetap di
  nilai TAKNOL yang jauh lebih besar di steady-state (syarat
  kesetimbangan -Kp*y+Ki*xI=0, yaitu xI_ss=Kp*y_ref/Ki). Tanpa
  batas baru, integrator akan mentok sebelum sempat mencapai
  nilai yang dibutuhkan, menyebabkan GALAT TUNAK PERMANEN
  (dibuktikan lewat simulasi roll: mentok di 7,55 deg, bukan
  mencapai referensi 12 deg, saat memakai batas lama).

  SEMUA PARAMETER LAIN (fisik, gain PID Kp/Ki/Kd, noise sensor,
  seed, dll.) TIDAK BERUBAH SAMA SEKALI dari v3 -- HANYA batas
  saturasi integrator (Bagian 11b) dan blok verifikasi (Bagian 16)
  yang diubah/ditambahkan.

  WAJIB DITERAPKAN JUGA DI: seluruh skrip step_response_*.py,
  servo_mix.py (jika memuat logika PID), dan blok PID Simulink --
  lihat catatan lengkap di akhir berkas ini.
================================================================
"""
import numpy as np

# ─── 1. PARAMETER FISIK WAHANA (acuan D=150mm, L=320mm) ──────
m   = 0.5            # Massa total [kg]
g   = 9.81           # Gravitasi [m/s²]
R_body = 0.075       # Radius badan [m] (Ø150 mm)
L_body = 0.32        # Panjang badan [m]
Ixx = (1/12)*m*(3*R_body**2 + L_body**2)   # 4,9698e-3
Iyy = Ixx                                    # silinder simetris
Izz = 0.5*m*R_body**2                        # 1,40625e-3

# ─── 2. MOTOR & THRUST (A2212/6T 2200KV, 3S 11,1V) ──────────
KV        = 2200
V_batt    = 11.1
omega_max = 0.8 * KV * V_batt * 2*np.pi/60   # 2045,8 rad/s
thrust_max = 0.850 * g                        # 0,850 kgf → N
kT        = thrust_max / omega_max**2         # 1,9923e-6

# ─── 3. AERODINAMIKA ────────────────────────────────────────
r_prop = 0.0635       # Radius prop 5" [m]
rho    = 1.225        # Kerapatan udara [kg/m³]
b_aero = 1.0e-3       # Redaman aerodinamis rotasi [N·m·s/rad]

# ─── 4. BATAS AKTUATOR ──────────────────────────────────────
servo_max_deg = 30
servo_max     = np.deg2rad(servo_max_deg)    # 0,5236 rad
servo_min     = -servo_max

# ─── 5. EFEKTIVITAS TORSI SIRIP (otoritas kontrol FISIK) ────
ddphi_max = 12.0                              # otoritas target [rad/s²]
Ks_hover  = Ixx * ddphi_max / servo_max        # 0,1139 N·m/rad
Keff      = Ks_hover     # alias kompatibilitas mundur
Kplant_v2 = Keff / Ixx    # [v2, TIDAK dipakai lagi utk PID] 22,92

# ─── 5b. TORSI REAKSI PROPELLER & FEEDFORWARD YAW ───────────
kappa_y  = 0.5          # faktor efektivitas mixing kolektif yaw [-]
Ky_hover = kappa_y * Ks_hover   # 0,05695 N·m/rad
c_q      = 0.005        # koefisien torsi reaksi propeller [m]

# ─── 5c. EFEKTIVITAS SIRIP TERHADAP PROP WASH: w(omega) ─────
def w_omega(omega, omega_hover_ref):
    """Faktor efektivitas sirip vs kecepatan rotor aktual."""
    return max((omega/omega_hover_ref)**2, 0.05)   # guard div-kecil

# ─── 6. KONDISI HOVER (dihitung dari fisika) ────────────────
omega_hover    = np.sqrt((m*g)/kT)            # 1569,1 rad/s
throttle_hover = omega_hover/omega_max        # 0,76696
q_dyn_hover    = 0.5*rho*(omega_hover*r_prop)**2

Q_hover = c_q * m * g                          # 0,02453 N.m
yaw_ff  = Q_hover / (4*Ky_hover)               # 0,1077 rad (~6,17 deg)

# ─── 7. GAIN PLANT TORSI-DOMAIN ──────────────────────────────
alpha = 5.0             # rasio kutub tambahan thd wn
b_roll  = 1/Ixx          # 201,22 rad/s² per N·m
b_pitch = 1/Iyy          # = b_roll
b_yaw   = 1/Izz          # 711,11 rad/s² per N·m
b_alt   = 1/m            # 2,00 m/s² per N

def pole_placement_pid(b, wn, zeta, alpha):
    """Kp,Ki,Kd dari penempatan kutub (s^2+2*zeta*wn*s+wn^2)(s+alpha*wn)=0,
    untuk plant integrator-ganda dgn gain b (Bab II Subbab pole placement)."""
    Kd = (alpha + 2*zeta) * wn / b
    Kp = (1 + 2*alpha*zeta) * wn**2 / b
    Ki = alpha * wn**3 / b
    return Kp, Ki, Kd

# ─── 8. PID ROLL/PITCH (pole-placement PENUH: wn=3,5 zeta=0,85) ─
wn_rp, zeta_rp = 3.5, 0.85
Kp_roll, Ki_roll, Kd_roll = pole_placement_pid(b_roll, wn_rp, zeta_rp, alpha)
# Kp=0,57836  Ki=1,06540  Kd=0,11654 -- TIDAK BERUBAH dari v3
pid_roll_outmax =  servo_max     # catatan: batas SUDUT SIRIP (di servo_mix.py,
pid_roll_outmin = -servo_max     #   BUKAN batas keluaran PID torsi file ini)
int_roll_limit  = servo_max * 0.6   # [v2 legacy, TIDAK dipakai lagi -- lihat 11b]

# ─── 9. PID PITCH (identik roll, sumbu simetris) ────────────
Kp_pitch, Ki_pitch, Kd_pitch = Kp_roll, Ki_roll, Kd_roll
pid_pitch_outmax =  servo_max
pid_pitch_outmin = -servo_max
int_pitch_limit  = servo_max * 0.6   # [v2 legacy, TIDAK dipakai lagi -- lihat 11b]

# ─── 10. PID YAW (pole-placement PENUH) ─────────────────────
Kp_yaw, Ki_yaw, Kd_yaw = pole_placement_pid(b_yaw, wn_rp, zeta_rp, alpha)
# Kp=0,16365  Ki=0,30146  Kd=0,03298 -- TIDAK BERUBAH dari v3
Kplant_yaw_v2 = 4*Keff*kappa_y/Izz             # [v2, disimpan utk telusur] ~162,0
pid_yaw_outmax  =  np.deg2rad(20)
pid_yaw_outmin  = -np.deg2rad(20)
int_yaw_limit   = servo_max * 0.4    # [v2 legacy, TIDAK dipakai lagi -- lihat 11b]

# ─── 11. PID ALTITUDE (pole-placement PENUH + konversi throttle) ─
wn_a, zeta_a = 1.5, 0.9
Kp_alt, Ki_alt, Kd_alt = pole_placement_pid(b_alt, wn_a, zeta_a, alpha)
# Kp=11,250  Ki=8,4375  Kd=5,100 -- TIDAK BERUBAH dari v3
Kalt_v2 = kT * omega_max**2 / m                # [v2, disimpan utk telusur] ~16,68

F_to_throttle = 1 / (2 * kT * omega_hover * omega_max)   # ~1/12,79 per Newton

pid_alt_outmax =  0.40   # batas Delta THROTTLE (setelah F_to_throttle)
pid_alt_outmin = -0.40
int_alt_limit  = 0.3     # [v2 legacy, TIDAK dipakai lagi -- lihat 11b]
N_filter = 100            # Filter derivatif [rad/s]

# ─── 11a. BATAS KELUARAN PID TORSI-DOMAIN (TIDAK BERUBAH dari v3) ─
# Batas fisik aktuator (anti-windup terluar) -- independen dari
# bentuk PID (galat vs umpan-balik-keadaan).
pid_roll_torque_max  = 2 * Ks_hover * servo_max          # ~0,11927 N.m
pid_pitch_torque_max = pid_roll_torque_max                # Ixx=Iyy -> identik
pid_yaw_torque_max   = 4 * Ky_hover * pid_yaw_outmax       # ~0,07952 N.m
pid_alt_force_max    = pid_alt_outmax / F_to_throttle      # ~5,1163 N

# ─── 11b. BATAS SATURASI INTEGRATOR -- DIHITUNG ULANG (PERBAIKAN v4) ─
#
#   Bentuk umpan-balik-keadaan: u = -Kp*y - Kd*dy + Ki*xI
#   Syarat kesetimbangan (dy=0, y=y_ref, u=0):
#       0 = -Kp*y_ref + Ki*xI_ss   -->   xI_ss = Kp*y_ref/Ki
#   Batas integrator HARUS mampu menampung xI_ss pada REFERENSI
#   TERBESAR yang mungkin diberikan pada tiap sumbu, ditambah
#   margin keamanan supaya tidak mentok tepat di titik kerja.
#
#   Nilai INI (bukan int_*_limit Bagian 8-11) yang WAJIB dipakai
#   pada skrip step_response_*.py dan blok Integrator Saturation
#   Simulink.
#
#   Referensi terbesar yang diasumsikan (SESUAIKAN bila skenario
#   pengujianmu berbeda):
#     - roll/pitch/yaw : servo_max (30 deg)
#     - altitude       : 5 m (margin di atas hover nominal 3 m)
MARGIN_INT = 1.5   # margin keamanan 50% di atas kebutuhan minimum

roll_ref_max_uji = servo_max        # 30 deg
yaw_ref_max_uji  = servo_max        # 30 deg
alt_ref_max_uji  = 5.0              # meter

pid_roll_int_max  = (Kp_roll * roll_ref_max_uji / Ki_roll) * MARGIN_INT   # ~0,4264
pid_pitch_int_max = pid_roll_int_max                                      # identik (Ixx=Iyy)
pid_yaw_int_max   = (Kp_yaw  * yaw_ref_max_uji  / Ki_yaw)  * MARGIN_INT   # ~0,4264
pid_alt_int_max   = (Kp_alt  * alt_ref_max_uji  / Ki_alt)  * MARGIN_INT   # ~10,00

# ─── 12. KALMAN FILTER ──────────────────────────────────────
dt    = 0.005
Ad_kf = np.array([[1, dt], [0, 1]])
Bd_roll_kf  = np.array([dt**2/2, dt]) * b_roll     # ~[2,515e-3; 1,006]
Bd_pitch_kf = Bd_roll_kf.copy()                     # Ixx=Iyy
C_kf  = np.array([[1, 0]])

Q_kf  = np.diag([0.001, 0.003])
R_kf  = 0.03
P0_kf = np.eye(2)

# ─── 13. WAKTU SIMULASI ─────────────────────────────────────
Ts    = dt
t_end = 28.0   # 28 s, 13 fase (Bab III Tabel skenario)

# ─── 14. SETPOINT DEFAULT ───────────────────────────────────
roll_ref_default  = 0
pitch_ref_default = 0
yaw_ref_default   = 0
alt_ref_default   = 0

# ─── 15. NOISE SENSOR (GY-87) ───────────────────────────────
gyro_noise_pwr  = 1e-6
accel_noise_pwr = 1e-4
gyro_noise_sim  = 1e-7
accel_noise_sim = 4e-6

noise_seed_gx = 23341
noise_seed_gy = 23342
noise_seed_ar = 23343
noise_seed_ap = 23344


# ═══════════════════════════════════════════════════════════
# 16. VERIFIKASI -- BENTUK YANG SEBENARNYA DIPAKAI (BARU v4)
# ═══════════════════════════════════════════════════════════
def _verifikasi_state_feedback(b, Kp, Ki, Kd, t_max=8.0):
    """Overshoot/settling-time/pole dari matriks ruang-keadaan
    lingkar tertutup BENTUK UMPAN-BALIK-KEADAAN (Persamaan
    matriks_lingkar_tertutup BAB III) -- BUKAN dari feedback(L,1)
    bentuk galat yang menyesatkan (lihat riwayat perbaikan di
    kepala berkas)."""
    from scipy import signal
    Acl = np.array([[0, 1, 0],
                     [-b*Kp, -b*Kd, b*Ki],
                     [-1, 0, 0]])
    Bcl = np.array([[0], [0], [1]])
    Ccl = np.array([[1, 0, 0]])
    sys = signal.StateSpace(Acl, Bcl, Ccl, 0)
    t = np.linspace(0, t_max, 200000)
    t, y = signal.step(sys, T=t)
    peak = y.max()
    os_pct = max(0.0, (peak - 1.0) * 100)
    ss = y[-1]
    band = 0.02
    idx = np.where(np.abs(y - 1.0) > band)[0]
    ts = t[idx[-1]] if len(idx) > 0 else 0.0
    poles = np.linalg.eigvals(Acl)
    return os_pct, ts, poles


def _verifikasi_pm_gm(b, Kp, Ki, Kd, N=100.0):
    """PM/GM dari L(s)=C(s)G(s) dengan turunan terfilter N=100,
    SAMA seperti blok PID Simulink yang sebenarnya diimplementasikan."""
    def L(w):
        s = 1j * w
        C = Kp + Ki / s + Kd * N * s / (s + N)
        G = b / s**2
        return C * G
    w = np.logspace(-1, 3, 500000)
    magdB = 20 * np.log10(np.abs(L(w)))
    idx = np.where((magdB[:-1] > 0) & (magdB[1:] <= 0))[0]
    if len(idx) == 0:
        return np.nan, np.nan
    wc = np.interp(0, [magdB[idx[0]+1], magdB[idx[0]]], [w[idx[0]+1], w[idx[0]]])
    pm = 180 + np.degrees(np.angle(L(wc)))
    return wc, pm


if __name__ == "__main__":
    print("=== Parameter Monochopter v4 (Python, PID umpan-balik-keadaan) ===")
    print(f"  Massa          : {m:.3f} kg")
    print(f"  Ixx=Iyy        : {Ixx:.4e} | Izz : {Izz:.4e}")
    print(f"  omega_max      : {omega_max:.1f} rad/s | omega_hover : {omega_hover:.1f} rad/s")
    print(f"  kT             : {kT:.4e} | throttle_hover : {throttle_hover:.5f}")
    print(f"  Ks_hover       : {Ks_hover:.4f} N.m/rad | Ky_hover : {Ky_hover:.4f} N.m/rad")
    print(f"  Q_hover        : {Q_hover:.4f} N.m | yaw_ff : {yaw_ff:.4f} rad ({np.rad2deg(yaw_ff):.2f} deg)")
    print("  --- Gain plant torsi-domain ---")
    print(f"  b_roll=b_pitch : {b_roll:.2f} | b_yaw : {b_yaw:.2f} | b_alt : {b_alt:.2f}")
    print(f"  alpha (pole placement) : {alpha:.1f}")
    print(f"  PID Roll/Pitch : Kp={Kp_roll:.5f} Ki={Ki_roll:.5f} Kd={Kd_roll:.5f}")
    print(f"  PID Yaw        : Kp={Kp_yaw:.5f} Ki={Ki_yaw:.5f} Kd={Kd_yaw:.5f}")
    print(f"  PID Alt        : Kp={Kp_alt:.4f} Ki={Ki_alt:.4f} Kd={Kd_alt:.4f}")
    print(f"  F_to_throttle  : {F_to_throttle:.6f} per Newton (1/{1/F_to_throttle:.3f})")
    print("  --- Batas keluaran PID torsi-domain ---")
    print(f"  pid_roll/pitch_torque_max : {pid_roll_torque_max:.5f} N.m")
    print(f"  pid_yaw_torque_max        : {pid_yaw_torque_max:.5f} N.m")
    print(f"  pid_alt_force_max         : {pid_alt_force_max:.4f} N")
    print("  --- Batas integrator (bentuk umpan-balik-keadaan, BARU v4) ---")
    print(f"  pid_roll_int_max  : {pid_roll_int_max:.4f}  (thd referensi maks {np.rad2deg(roll_ref_max_uji):.1f} deg)")
    print(f"  pid_pitch_int_max : {pid_pitch_int_max:.4f}  (identik roll, Ixx=Iyy)")
    print(f"  pid_yaw_int_max   : {pid_yaw_int_max:.4f}  (thd referensi maks {np.rad2deg(yaw_ref_max_uji):.1f} deg)")
    print(f"  pid_alt_int_max   : {pid_alt_int_max:.4f}  (thd referensi maks {alt_ref_max_uji:.1f} m)")
    print(f"  Sample time    : {Ts*1000:.1f} ms ({1/dt:.0f} Hz)")

    try:
        print("\n--- Verifikasi step response (bentuk umpan-balik-keadaan) ---")
        for nm, b, Kp, Ki, Kd in [
            ('roll', b_roll, Kp_roll, Ki_roll, Kd_roll),
            ('yaw',  b_yaw,  Kp_yaw,  Ki_yaw,  Kd_yaw),
            ('alt',  b_alt,  Kp_alt,  Ki_alt,  Kd_alt),
        ]:
            os_pct, ts, poles = _verifikasi_state_feedback(b, Kp, Ki, Kd,
                                                             t_max=10.0 if nm == 'alt' else 8.0)
            wc, pm = _verifikasi_pm_gm(b, Kp, Ki, Kd)
            lhp = all(p.real < 0 for p in poles)
            print(f"  {nm:5s} : PM={pm:.2f} deg | OS={os_pct:.3f}% | ts={ts:.3f}s | "
                  f"semua pole LHP? {lhp}")
        print("(Overshoot roll/yaw seharusnya ~0,61% dan alt ~0,15% sesuai spesifikasi zeta desain)")
    except ImportError:
        print("\n[scipy tidak tersedia -- verifikasi step response dilewati]")

    print("\nPENTING: servo_mix.py, kalman_update.py, step_response_*.py, dan")
    print("blok PID Simulink HARUS memakai bentuk umpan-balik-keadaan")
    print("(u=-Kp*y-Kd*dy+Ki*xI) dgn pid_*_int_max BARU di atas -- lihat")
    print("riwayat perbaikan di kepala berkas ini.")
    print("=============================================================")