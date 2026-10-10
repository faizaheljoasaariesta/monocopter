"""
============================================================
FILE: simulasi_full_system.py
TUJUAN: Simulasi penuh closed-loop + export hasil
SKENARIO: Takeoff → Hover → Roll → Pitch → Yaw → Landing
============================================================
Versi v5 -- PERBAIKAN BENTUK KENDALI (dari v4).

RIWAYAT PERBAIKAN v4 -> v5:

  BUG 1 (bentuk PID): seluruh PID (roll/pitch/yaw/altitude)
    sebelumnya memakai bentuk PID-PADA-GALAT (u=Kp*e+Ki*int(e)+
    Kd*de/dt). Diperbaiki ke bentuk UMPAN-BALIK-KEADAAN yang
    benar (Persamaan hukum_kendali_state_feedback Bab II):
    u=-Kp*y-Kd*dy+Ki*xI. Kedua bentuk punya kutub lingkar tertutup
    identik, tapi bentuk galat punya zero tambahan yang membesarkan
    overshoot jauh di atas prediksi zeta rancangan.

  BUG 2 (estimasi laju Kalman terbuang): xKr[1]/xKp[1] (estimasi
    laju roll/pitch tersaring Kalman) SEBELUMNYA dihitung tapi
    tidak pernah dipakai. Untuk bentuk umpan-balik-keadaan yang
    baru, estimasi laju ini WAJIB dipakai sebagai "dy" (bukan
    turunan-mundur dari galat) -- inilah justru salah satu manfaat
    Kalman Filter yang seharusnya dipakai (estimasi bebas-derau
    utk suku turunan, bukan cuma suku proporsional).

  BUG 3 (batas integrator legacy): int_roll_limit/int_pitch_limit/
    int_yaw_limit/int_alt_limit (v2, dikalibrasi utk bentuk galat)
    diganti pid_roll_int_max/pid_pitch_int_max/pid_yaw_int_max/
    pid_alt_int_max (v4, dikalibrasi utk bentuk umpan-balik-keadaan
    -- lihat monochopter_params_v4.py Bagian 11b).

  BUG 4 (satuan clamp keluaran PID salah): uR/uP SEBELUMNYA
    diclamp ke +-servo_max (RADIAN, satuan SUDUT SIRIP), padahal
    uR/uP adalah TORSI (N.m). uY diclamp ke pid_yaw_outmax (RADIAN
    juga). uA diclamp ke pid_alt_outmax=0,40 (FRAKSI THROTTLE),
    padahal uA adalah GAYA (Newton). Diperbaiki ke
    pid_roll_torque_max/pid_pitch_torque_max/pid_yaw_torque_max/
    pid_alt_force_max (satuan torsi/gaya yang benar).

  BUG 5 (konversi throttle hilang, PALING SERIUS): baris
    "throttle = throttle_hover + uA" menjumlahkan uA (Newton)
    LANGSUNG ke throttle_hover TANPA dikalikan F_to_throttle,
    seolah-olah uA sudah berupa Delta throttle. Diperbaiki menjadi
    "throttle = throttle_hover + uA*F_to_throttle" sesuai Persamaan
    konversi_throttle Bab III.

  Yaw memerlukan penanganan KHUSUS pada bentuk umpan-balik-keadaan
  karena sifat "wrap-around" sudut heading (referensi bisa dilewati
  dari sisi manapun). Solusinya: turunkan psi_eff = yawRef - eY,
  dengan eY = wrap(yawRef-psi) sudah dihitung dgn benar (mekanisme
  wrap LAMA dipertahankan), sehingga psi_eff merepresentasikan psi
  "terbuka-gulung" konsisten di sekitar yawRef, tanpa lompatan.

  SEMUA logika fisika (tau_roll/tau_pitch/tau_yaw dari s1..s4,
  servo mixer, gain scheduling) TIDAK diubah -- sudah benar sejak
  v4. Skenario, ekspor CSV/NPZ, dan metrik evaluasi juga TIDAK
  diubah strukturnya (angka yang dilaporkan otomatis mengikuti
  hasil simulasi yang sudah diperbaiki).
============================================================
"""

import numpy as np
import time, csv, os
from monocopter_params import *

# ─── PATH OUTPUT (relatif terhadap lokasi skrip) ────────────
HERE = os.path.dirname(os.path.abspath(__file__))
OUTDIR = os.path.join(HERE, 'hasil-simulasi')
os.makedirs(OUTDIR, exist_ok=True)

rng_gx = np.random.RandomState(noise_seed_gx)
rng_gy = np.random.RandomState(noise_seed_gy)
rng_ar = np.random.RandomState(noise_seed_ar)
rng_ap = np.random.RandomState(noise_seed_ap)

def clamp(val, lo, hi):
    return float(np.clip(val, lo, hi))

# ─── SCENARIO (identik dengan MATLAB) ───────────────────────
# (t_start, roll_deg, pitch_deg, yaw_deg, alt_m)

scenario = np.array([
    [ 0.0,   0,   0,    0,  0.0],   # Ground idle
    [ 2.0,   0,   0,    0,  3.0],   # Takeoff ke 3 m
    [ 5.0,   0,   0,    0,  3.0],   # Hover stabil
    [ 7.0,  12,   0,    0,  3.0],   # Manuver roll +12°
    [10.0,   0,   0,    0,  3.0],   # Level recovery
    [12.0,   0,  10,    0,  3.0],   # Manuver pitch +10°
    [15.0,   0,   0,    0,  3.0],   # Level recovery
    [17.0,   0,   0,   30,  3.0],   # Yaw turn ke heading 30°
    [19.5,   0,   0,   30,  3.0],   # Hold heading
    [21.0,   0,   0,    0,  3.0],   # Yaw return
    [22.0,   0,   0,    0,  2.0],   # Descend ke 2 m
    [24.0,   0,   0,    0,  1.0],   # Descend ke 1 m
    [25.5,   0,   0,    0, -0.3],   # Sentuh tanah (target sedikit negatif)
    [28.0,   0,   0,    0, -0.3],   # Landing tuntas
])

t_total = 28.0
N = round(t_total / dt)
t_vec = np.arange(N) * dt

def interp_prev(t_q, t_s, vals):
    out = np.zeros(len(t_q))
    for i, t in enumerate(t_q):
        idx = int(np.searchsorted(t_s, t, side='right')) - 1
        out[i] = vals[max(0, min(idx, len(vals)-1))]
    return out

rollRef_vec  = interp_prev(t_vec, scenario[:,0], np.deg2rad(scenario[:,1]))
pitchRef_vec = interp_prev(t_vec, scenario[:,0], np.deg2rad(scenario[:,2]))
yawRef_vec   = interp_prev(t_vec, scenario[:,0], np.deg2rad(scenario[:,3]))
altRef_vec   = interp_prev(t_vec, scenario[:,0], scenario[:,4])

X         = np.zeros((8, N))
U         = np.zeros((4, N))
PHI_EST   = np.zeros(N)
THETA_EST = np.zeros(N)
DPHI_EST  = np.zeros(N)   # BARU (v5): dicatat utk pembanding Bab IV
DTHETA_EST = np.zeros(N)  # BARU (v5)
THR       = np.zeros(N)
REF       = np.zeros((4, N))
POS       = np.zeros((3, N))
W_LOG     = np.zeros(N)      # log w(omega) utk verifikasi
GYRO_X_LOG = np.zeros(N)     # giroskop direkam utk pembanding Bab IV
GYRO_Y_LOG = np.zeros(N)     #   TIDAK dipakai sbg masukan Kalman

x_state = np.zeros(8)
int_r=0.0; int_p=0.0; int_y=0.0; int_a=0.0
# pe_r/pe_p/pe_y/pe_a (memori galat utk turunan-mundur) DIHAPUS --
# bentuk umpan-balik-keadaan v5 memakai dphiEst/dthetaEst (Kalman)
# dan dpsi/dz (state langsung) sbg "dy", bukan turunan-mundur galat.

xKr = np.zeros(2); PKr = np.eye(2)
xKp = np.zeros(2); PKp = np.eye(2)
A_kf = np.array([[1.0, dt],[0.0, 1.0]])
C_kf = np.array([1.0, 0.0])
B_roll_kf  = np.array([dt**2/2*b_roll,  dt*b_roll])
B_pitch_kf = np.array([dt**2/2*b_pitch, dt*b_pitch])
tau_roll_prev  = 0.0
tau_pitch_prev = 0.0

pos_w = np.zeros(3)
vel_w = np.zeros(3)

print(f"Menjalankan simulasi {t_total:.0f} detik ({N} steps)...")
t0 = time.time()

for k in range(N):
    rollRef  = rollRef_vec[k]
    pitchRef = pitchRef_vec[k]
    yawRef   = yawRef_vec[k]
    altRef   = altRef_vec[k]
    REF[:, k] = [rollRef, pitchRef, yawRef, altRef]

    # State saat ini
    phi_s  = x_state[0]; dphi   = x_state[1]
    theta  = x_state[2]; dtheta = x_state[3]
    psi_s  = x_state[4]; dpsi   = x_state[5]
    z_cur  = max(0.0, x_state[6]); dz_cur = x_state[7]

    # ─── IMU noise (giroskop TETAP direkam utk pembanding Bab IV,
    #     TIDAK dipakai sbg masukan Kalman) ──────────────────────
    gyroX_logged = dphi   + np.sqrt(gyro_noise_sim/dt) * rng_gx.randn()
    gyroY_logged = dtheta + np.sqrt(gyro_noise_sim/dt) * rng_gy.randn()
    accR  = phi_s  + np.sqrt(accel_noise_sim/dt) * rng_ar.randn()
    accP  = theta  + np.sqrt(accel_noise_sim/dt) * rng_ap.randn()

    # ─── Kalman — roll (masukan torsi tau_roll_prev, causal) ────
    xpR = A_kf @ xKr + B_roll_kf * tau_roll_prev
    PpR = A_kf @ PKr @ A_kf.T + Q_kf
    Sr  = float(C_kf @ PpR @ C_kf) + R_kf
    Kr  = (PpR @ C_kf) / Sr
    xKr = xpR + Kr * (accR - float(C_kf @ xpR))
    PKr = (np.eye(2) - np.outer(Kr, C_kf)) @ PpR
    phiEst  = float(xKr[0])
    dphiEst = float(xKr[1])   # BARU (v5): estimasi laju roll -- SEBELUMNYA dibuang

    # ─── Kalman — pitch (masukan torsi tau_pitch_prev) ──────────
    xpP = A_kf @ xKp + B_pitch_kf * tau_pitch_prev
    PpP = A_kf @ PKp @ A_kf.T + Q_kf
    Sp  = float(C_kf @ PpP @ C_kf) + R_kf
    Kpk = (PpP @ C_kf) / Sp
    xKp = xpP + Kpk * (accP - float(C_kf @ xpP))
    PKp = (np.eye(2) - np.outer(Kpk, C_kf)) @ PpP
    thetaEst  = float(xKp[0])
    dthetaEst = float(xKp[1])  # BARU (v5): estimasi laju pitch -- SEBELUMNYA dibuang

    # ─── PID -- BENTUK UMPAN-BALIK-KEADAAN (PERBAIKAN UTAMA v5) ──
    # u = -Kp*y - Kd*dy + Ki*xI. Roll/pitch memakai y,dy dari
    # ESTIMASI KALMAN (phiEst/dphiEst, thetaEst/dthetaEst) --
    # bukan state fisik langsung, karena pada sistem nyata memang
    # hanya estimasi inilah yang tersedia bagi pengendali. Yaw dan
    # altitude memakai state langsung (psi_s/dpsi, z_cur/dz_cur)
    # karena keduanya tidak diestimasi Kalman sesuai arsitektur
    # Bab III (Subbab DKF hanya mencakup roll/pitch).

    # Roll
    eR    = rollRef - phiEst
    int_r = clamp(int_r + eR*dt, -pid_roll_int_max, pid_roll_int_max)
    uR    = clamp(-Kp_roll*phiEst - Kd_roll*dphiEst + Ki_roll*int_r,
                  -pid_roll_torque_max, pid_roll_torque_max)

    # Pitch
    eP    = pitchRef - thetaEst
    int_p = clamp(int_p + eP*dt, -pid_pitch_int_max, pid_pitch_int_max)
    uP    = clamp(-Kp_pitch*thetaEst - Kd_pitch*dthetaEst + Ki_pitch*int_p,
                  -pid_pitch_torque_max, pid_pitch_torque_max)

    # Yaw (wrap-safe: psi_eff = yawRef - eY_wrapped merepresentasikan
    # psi "terbuka-gulung" konsisten di sekitar yawRef, tanpa lompatan
    # +-360 deg saat referensi dilewati dari sisi manapun)
    eY_wrapped = float(((yawRef - psi_s + np.pi) % (2*np.pi)) - np.pi)
    psi_eff    = yawRef - eY_wrapped
    int_y = clamp(int_y + eY_wrapped*dt, -pid_yaw_int_max, pid_yaw_int_max)
    uY    = clamp(-Kp_yaw*psi_eff - Kd_yaw*dpsi + Ki_yaw*int_y,
                  -pid_yaw_torque_max, pid_yaw_torque_max)

    # Altitude
    eA    = altRef - z_cur
    int_a = clamp(int_a + eA*dt, -pid_alt_int_max, pid_alt_int_max)
    uA    = clamp(-Kp_alt*z_cur - Kd_alt*dz_cur + Ki_alt*int_a,
                  -pid_alt_force_max, pid_alt_force_max)

    # ─── THROTTLE (PERBAIKAN v5: WAJIB dikali F_to_throttle -- uA
    #     bersatuan NEWTON, bukan Delta throttle langsung) ────────
    sMax = servo_max
    throttle = clamp(throttle_hover + uA*F_to_throttle, 0.05, 1.0)

    # ─── GAIN SCHEDULING: efektivitas sirip vs throttle ──────────
    omega_for_w = throttle * omega_max
    w = w_omega(omega_for_w, omega_hover)
    uR_c = uR / w
    uP_c = uP / w
    uY_c = uY / w
    W_LOG[k] = w

    # ─── Servo mixer (konversi torsi-ke-defleksi, TIDAK diubah) ──
    delta_r = uR_c / (2*Ks_hover)
    delta_p = uP_c / (2*Ks_hover)
    delta_y = uY_c / (4*Ky_hover)

    s1 = clamp(delta_r + delta_y + yaw_ff, -sMax, sMax)
    s2 = clamp(delta_p + delta_y + yaw_ff, -sMax, sMax)
    s3 = clamp(-delta_r + delta_y + yaw_ff, -sMax, sMax)
    s4 = clamp(-delta_p + delta_y + yaw_ff, -sMax, sMax)

    # ─── Physics (TIDAK diubah, sudah benar sejak v4) ────────────
    omega = throttle * omega_max
    FT    = kT * omega**2

    Q_reaction = c_q * FT

    tau_roll  = Ks_hover*w*(s1 - s3)             - b_aero*dphi
    tau_pitch = Ks_hover*w*(s2 - s4)             - b_aero*dtheta
    tau_yaw   = Ks_hover*w*kappa_y*(s1+s2+s3+s4) - Q_reaction - b_aero*dpsi

    tau_roll_prev  = tau_roll
    tau_pitch_prev = tau_pitch

    ddph = ((Iyy - Izz)*dtheta*dpsi + tau_roll)  / Ixx
    ddth = ((Izz - Ixx)*dphi  *dpsi + tau_pitch) / Iyy
    ddps = ((Ixx - Iyy)*dphi*dtheta  + tau_yaw)  / Izz
    ddz  = -g + np.cos(phi_s)*np.cos(theta)*FT/m

    # ─── Integrasi ──────────────────────────────────────────────
    x_state[0] = x_state[0] + dt*dphi
    x_state[1] = x_state[1] + dt*ddph
    x_state[2] = x_state[2] + dt*dtheta
    x_state[3] = x_state[3] + dt*ddth
    x_state[4] = x_state[4] + dt*dpsi
    x_state[5] = x_state[5] + dt*ddps
    x_state[6] = max(0.0, x_state[6] + dt*dz_cur)
    x_state[7] = dz_cur + dt*ddz
    if x_state[6] == 0.0:
        x_state[7] = max(0.0, x_state[7])

    # ─── World XY (emergent dari kemiringan) ─────────────────────
    ax_w =  float(np.sin(x_state[2])) * FT / m
    ay_w = -float(np.sin(x_state[0])) * FT / m
    vel_w[0] = vel_w[0] + (ax_w - 1.5*vel_w[0])*dt
    vel_w[1] = vel_w[1] + (ay_w - 1.5*vel_w[1])*dt
    pos_w[0] += vel_w[0]*dt
    pos_w[1] += vel_w[1]*dt
    pos_w[2]  = x_state[6]

    X[:, k]      = x_state
    U[:, k]      = np.array([s1,s2,s3,s4]) * 180.0/np.pi
    PHI_EST[k]   = phiEst
    THETA_EST[k] = thetaEst
    DPHI_EST[k]  = dphiEst
    DTHETA_EST[k] = dthetaEst
    THR[k]       = throttle * 100.0
    POS[:, k]    = pos_w.copy()
    GYRO_X_LOG[k] = gyroX_logged
    GYRO_Y_LOG[k] = gyroY_logged

print(f"Selesai dalam {time.time()-t0:.2f} detik CPU.")

# ─── METRIK (window konsisten dengan MATLAB) ────────────────
print('\n======= SYSTEM PERFORMANCE METRICS =======')
idx_roll = (t_vec >= 8) & (t_vec <= 9.8)
roll_err = np.abs(X[0,idx_roll]*180/np.pi - REF[0,idx_roll]*180/np.pi)
print(f'Roll Manuver:')
print(f'  Max Error  : {np.max(roll_err):.2f} deg')
print(f'  Mean Error : {np.mean(roll_err):.2f} deg')
print(f'  RMSE       : {np.sqrt(np.mean(roll_err**2)):.2f} deg')

idx_hov = (t_vec >= 5) & (t_vec <= 21)
alt_err = np.abs(X[6,idx_hov] - REF[3,idx_hov])
print(f'Altitude Hold:')
print(f'  Max Error  : {np.max(alt_err):.3f} m')
print(f'  Mean Error : {np.mean(alt_err):.3f} m')
print(f'  Std Dev    : {np.std(alt_err):.4f} m')

idx_yaw = (t_vec >= 17.5) & (t_vec <= 19.5)
yaw_err = np.abs(((X[4,idx_yaw] - REF[2,idx_yaw] + np.pi) % (2*np.pi) - np.pi)*180/np.pi)
print(f'Yaw Turn (hold heading 30 deg):')
print(f'  Max Error  : {np.max(yaw_err):.2f} deg')
print(f'  RMSE       : {np.sqrt(np.mean(yaw_err**2)):.2f} deg')
print(f'Gain scheduling w(omega): min={W_LOG.min():.3f} max={W_LOG.max():.3f}')

idx0 = np.searchsorted(t_vec, 7.0)
roll_step = X[0, idx0:]*180/np.pi
target = 12.0
band = max(0.02*abs(target), 0.5)
settled = np.where(np.abs(roll_step - target) < band)[0]
if len(settled) > 0:
    print(f'Roll Settling Time (2%): {settled[0]*dt:.3f} s')
print('==========================================')

# ─── EXPORT ──────────────────────────────────────────────────
sim_results = {
    'time':t_vec, 'phi_deg':X[0,:]*180/np.pi,
    'theta_deg':X[2,:]*180/np.pi, 'yaw_deg':X[4,:]*180/np.pi,
    'altitude_m':X[6,:], 'phi_est':PHI_EST*180/np.pi,
    'theta_est':THETA_EST*180/np.pi,
    'dphi_est':DPHI_EST, 'dtheta_est':DTHETA_EST,   # BARU (v5)
    'servo_deg':U,
    'throttle_pct':THR, 'ref_roll':REF[0,:]*180/np.pi,
    'ref_pitch':REF[1,:]*180/np.pi, 'ref_yaw':REF[2,:]*180/np.pi,
    'ref_alt':REF[3,:], 'x_pos':POS[0,:], 'y_pos':POS[1,:],
    'dphi':X[1,:], 'dtheta':X[3,:], 'dpsi':X[5,:], 'dz':X[7,:],
    'w_omega':W_LOG,
    'gyro_x_logged':GYRO_X_LOG,
    'gyro_y_logged':GYRO_Y_LOG,
    't_total':np.array([t_total]), 'scenario':scenario,
}
npz_path = os.path.join(OUTDIR, 'hasil_simulasi_full.npz')
np.savez(npz_path, **sim_results)
print(f'Data tersimpan ke: {npz_path}')

csv_path = os.path.join(OUTDIR, 'hasil_simulasi_full.csv')
with open(csv_path, 'w', newline='') as f:
    w_csv = csv.writer(f)
    w_csv.writerow(['time','roll_deg','pitch_deg','yaw_deg','alt_m',
                'phi_est','theta_est','throttle_pct','x_pos','y_pos',
                's1_deg','s2_deg','s3_deg','s4_deg','w_omega'])
    for i in range(N):
        w_csv.writerow([round(t_vec[i],4), round(X[0,i]*180/np.pi,4),
                    round(X[2,i]*180/np.pi,4), round(X[4,i]*180/np.pi,4),
                    round(X[6,i],4), round(PHI_EST[i]*180/np.pi,4),
                    round(THETA_EST[i]*180/np.pi,4), round(THR[i],3),
                    round(POS[0,i],4), round(POS[1,i],4),
                    round(U[0,i],3), round(U[1,i],3),
                    round(U[2,i],3), round(U[3,i],3), round(W_LOG[i],4)])
print(f'Data tersimpan ke: {csv_path}')