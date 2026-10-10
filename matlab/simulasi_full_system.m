clear; clc; monocopter_params;

%% --- KONFIGURASI SKENARIO TERBANG (13-FASE MISI) ------------
t_total = 28.0;                                                 % Total durasi [s]
N = round(t_total/dt);
t_vec = (0:N-1)*dt;

scenario = [
     0.0,   0,   0,   0,  0.0;                                  % 1. Start di tanah
     2.0,   0,   0,   0,  3.0;                                  % 2. Lepas landas (Takeoff)
     5.0,   0,   0,   0,  3.0;                                  % 3. Hover stabil 3m
     7.0,  12,   0,   0,  3.0;                                  % 4. Manuver Roll 12°
    10.0,   0,   0,   0,  3.0;                                  % 5. Level Recovery
    12.0,   0,  10,   0,  3.0;                                  % 6. Manuver Pitch 10° (t=12s)
    15.0,   0,   0,   0,  3.0;                                  % 7. Level Recovery
    17.0,   0,   0,  30,  3.0;                                  % 8. Manuver Yaw 30° (t=17s)
    19.5,   0,   0,  30,  3.0;                                  % 9. Hold Heading Yaw
    21.0,   0,   0,   0,  3.0;                                  % 10. Yaw Return
    22.0,   0,   0,   0,  2.0;                                  % 11. Descent Tahap 1 (ke 2 m)
    24.0,   0,   0,   0,  1.0;                                  % 12. Descent Tahap 2 (ke 1 m)
    25.5,   0,   0,   0, -0.3;                                  % 13. Touchdown
    28.0,   0,   0,   0, -0.3;                                  % 14. Selesai simulasi
];

% Interpolasi Setpoint secara kontinu
rollRef_vec  = interp1(scenario(:,1), scenario(:,2)*pi/180, t_vec, 'previous','extrap');
pitchRef_vec = interp1(scenario(:,1), scenario(:,3)*pi/180, t_vec, 'previous','extrap');
yawRef_vec   = interp1(scenario(:,1), scenario(:,4)*pi/180, t_vec, 'previous','extrap');
altRef_vec   = interp1(scenario(:,1), scenario(:,5),        t_vec, 'previous','extrap');

%% --- INISIALISASI STATE DAN LOG -----------------------------
X = zeros(8,N);                                                 % Vektor keadaan fisik [phi; dphi; theta; dtheta; psi; dpsi; z; dz]
U = zeros(4,N);                                                 % Sudut defleksi servo aktual [s1, s2, s3, s4] (deg)
PHI_EST  = zeros(1,N);                                          % Log estimasi Kalman Roll
THETA_EST= zeros(1,N);                                          % Log estimasi Kalman Pitch
THR = zeros(1,N);                                               % Log throttle motor (%)
GYRO_X_LOG = zeros(1,N);                                        % Log pembacaan gyro x (untuk visualisasi)
GYRO_Y_LOG = zeros(1,N);                                        % Log pembacaan gyro y (untuk visualisasi)
REF = zeros(4,N);                                               % Log referensi [rollRef, pitchRef, yawRef, altRef]
KERR= zeros(2,N);                                               % Log inovasi galat Kalman Filter

x = zeros(8,1);                                                 % Keadaan awal wahana (diam di tanah)

int_r = 0; int_p = 0; int_y = 0; int_a = 0;                     % State Integrator PID (xI)

xKr = [0;0]; PKr = eye(2);                                      % Keadaan Awal Kalman Filter (Roll & Pitch)
xKp = [0;0]; PKp = eye(2);

Ad_kf      = [1 dt; 0 1];                                       % Parameter Kalman Filter
B_roll_kf  = [dt^2/2 * b_roll;  dt * b_roll];
B_pitch_kf = [dt^2/2 * b_pitch; dt * b_pitch];
C_kf       = [1 0];

tau_roll_prev  = 0.0;                                           % Penyimpan torsi kendali siklus sebelumnya untuk Kalman Filter (causal)
tau_pitch_prev = 0.0;

%% --- BATAS SENSOR DAN LIMIT FISIK ---------------------------
MAX_ANG  = deg2rad(55);                                         % Batas kemiringan fisik wahana
MAX_RATE = deg2rad(250);                                        % Batas laju putaran fisik

gyro_std  = sqrt(gyro_noise_sim/dt);                            % Standar deviasi noise sensor (IMU)
accel_std = sqrt(accel_noise_sim/dt);

M_mix = [+1,  0, +1;                                            % Servo 1 (kiri)
          0, +1, +1;                                            % Servo 2 (depan)
         -1,  0, +1;                                            % Servo 3 (kanan)
          0, -1, +1];                                           % Servo 4 (belakang)

%% --- LOOP SIMULASI UTAMA ------------------------------------
fprintf('Menjalankan simulasi full system %.1f detik...\n', t_total);
tic;

for k = 1:N
    % 1. Ekstrak Keadaan Fisik Aktual
    phi    = clamp(x(1), -MAX_ANG, MAX_ANG);
    dphi   = clamp(x(2), -MAX_RATE, MAX_RATE);
    theta  = clamp(x(3), -MAX_ANG, MAX_ANG);
    dtheta = clamp(x(4), -MAX_RATE, MAX_RATE);
    psi    = x(5);
    dpsi   = clamp(x(6), -MAX_RATE, MAX_RATE);
    z      = max(0, x(7));
    dz     = x(8);
    
    X(:,k) = [phi; dphi; theta; dtheta; psi; dpsi; z; dz];      % Simpan keadaan fisik aktual ke log
    
    % 2. Ekstrak Referensi Saat Ini
    roll_ref  = rollRef_vec(k);
    pitch_ref = pitchRef_vec(k);
    yaw_ref   = yawRef_vec(k);
    alt_ref   = altRef_vec(k);
    REF(:,k)  = [roll_ref; pitch_ref; yaw_ref; alt_ref];
    
    % 3. Sensor Noisy (IMU MPU-6050)
    accel_roll_noisy  = phi    + accel_std * randn();
    accel_pitch_noisy = theta  + accel_std * randn();
    gyro_roll_noisy   = dphi   + gyro_std  * randn();
    gyro_pitch_noisy  = dtheta + gyro_std  * randn();
    
    GYRO_X_LOG(k) = gyro_roll_noisy;
    GYRO_Y_LOG(k) = gyro_pitch_noisy;
    
    % 4. Estimasi Sikap Kalman Filter (Roll & Pitch)
    % --- Kanal Roll KF ---
    xKr_pred = Ad_kf * xKr + B_roll_kf * tau_roll_prev;
    PKr_pred = Ad_kf * PKr * Ad_kf' + Q_kf;
    Sr = C_kf * PKr_pred * C_kf' + R_kf;
    Kr = PKr_pred * C_kf' / Sr;
    xKr = xKr_pred + Kr * (accel_roll_noisy - C_kf * xKr_pred);
    PKr = (eye(2) - Kr * C_kf) * PKr_pred;
    phi_est  = xKr(1);
    dphi_est = xKr(2);
    PHI_EST(k) = phi_est;
    KERR(1,k)  = accel_roll_noisy - phi_est;
    
    % --- Kanal Pitch KF ---
    xKp_pred = Ad_kf * xKp + B_pitch_kf * tau_pitch_prev;
    PKp_pred = Ad_kf * PKp * Ad_kf' + Q_kf;
    Sp = C_kf * PKp_pred * C_kf' + R_kf;
    Kp = PKp_pred * C_kf' / Sp;
    xKp = xKp_pred + Kp * (accel_pitch_noisy - C_kf * xKp_pred);
    PKp = (eye(2) - Kp * C_kf) * PKp_pred;
    theta_est  = xKp(1);
    dtheta_est = xKp(2);
    THETA_EST(k) = theta_est;
    KERR(2,k)    = accel_pitch_noisy - theta_est;
    
    % 5. Hukum Kendali Umpan-Balik-Keadaan (State Feedback PID)
    % --- Kendali Roll ---
    uR = -Kp_roll * phi_est - Kd_roll * dphi_est + Ki_roll * int_r;
    uR = clamp(uR, -pid_roll_torque_max, pid_roll_torque_max);
    int_r = int_r + (roll_ref - phi_est) * dt;
    int_r = clamp(int_r, -pid_roll_int_max, pid_roll_int_max);
    tau_roll_prev = uR;                                         % Simpan torsi untuk siklus Kalman berikutnya

    % --- Kendali Pitch ---
    uP = -Kp_pitch * theta_est - Kd_pitch * dtheta_est + Ki_pitch * int_p;
    uP = clamp(uP, -pid_pitch_torque_max, pid_pitch_torque_max);
    int_p = int_p + (pitch_ref - theta_est) * dt;
    int_p = clamp(int_p, -pid_pitch_int_max, pid_pitch_int_max);
    tau_pitch_prev = uP;                                        % Simpan torsi untuk siklus Kalman berikutnya
    
    % --- Kendali Yaw (Heading Tracking) ---
    uY = -Kp_yaw * psi - Kd_yaw * dpsi + Ki_yaw * int_y;
    uY = clamp(uY, -pid_yaw_torque_max, pid_yaw_torque_max);
    int_y = int_y + (yaw_ref - psi) * dt;
    int_y = clamp(int_y, -pid_yaw_int_max, pid_yaw_int_max);
    
    % --- Kendali Altitude ---
    uA = -Kp_alt * z - Kd_alt * dz + Ki_alt * int_a;
    uA = clamp(uA, -pid_alt_force_max, pid_alt_force_max);
    int_a = int_a + (alt_ref - z) * dt;
    int_a = clamp(int_a, -pid_alt_int_max, pid_alt_int_max);
    
    % 6. Konversi Gaya Kendali Altitude ke Delta Throttle & Throttle Total
    d_throttle = uA * F_to_throttle;
    d_throttle = clamp(d_throttle, -pid_out_max_a, pid_out_max_a);
    throttle = throttle_hover + d_throttle;
    throttle = clamp(throttle, 0, 1);
    THR(k) = throttle;
    
    % 7. Kompensasi Gain Scheduling Prop Wash w(omega)
    omega = throttle * omega_max;
    w_omega = (omega / omega_hover)^2;
    w_omega = max(w_omega, 0.05);                               % Proteksi pembagian nol di throttle sangat rendah

    % Torsi diubah ke defleksi sirip (radian) dengan kompensasi w(omega)
    delta_roll  = (uR / (2 * Ks_hover)) / w_omega;
    delta_pitch = (uP / (2 * Ks_hover)) / w_omega;
    delta_yaw   = (uY / (4 * Ky_hover)) / w_omega;
    
    % Matriks Pencampuran (Mixer Actuator) + Feedforward Trim Yaw
    s_raw = M_mix * [delta_roll; delta_pitch; delta_yaw] + yaw_ff;
    s_sat = clamp(s_raw, servo_min, servo_max);
    U(:,k) = s_sat * 180/pi;                                    % Simpan dalam derajat untuk analisis log
    
    % 8. Dinamika Fisik Wahana (Generasi Torsi & Gaya Nyata)
    s1 = s_sat(1); s2 = s_sat(2); s3 = s_sat(3); s4 = s_sat(4);
    
    % Torsi fisik aktual yang terbangkitkan
    tau_roll_act  = Ks_hover * w_omega * (s1 - s3);
    tau_pitch_act = Ks_hover * w_omega * (s2 - s4);
    
    % Torsi yaw aktual (Otoritas Sirip Kolektif - Torsi Reaksi Propeller)
    F_lift = kT * omega^2;
    Q_act  = c_q * F_lift;                                      % Torsi reaksi propeller proporsional terhadap thrust
    delta_yaw_act = (s1 + s2 + s3 + s4) / 4;
    tau_yaw_act   = 4 * Ky_hover * w_omega * delta_yaw_act - Q_act;
    
    % Percepatan Sudut dan Vertikal
    ddphi   = b_roll  * (tau_roll_act  - b_aero * dphi);
    ddtheta = b_pitch * (tau_pitch_act - b_aero * dtheta);
    ddpsi   = b_yaw   * (tau_yaw_act   - b_aero * dpsi);
    ddz     = b_alt   * (F_lift - m * g);
    
    % Integrasi Numerik Euler
    dphi   = dphi   + ddphi   * dt;
    phi    = phi    + dphi    * dt;
    dtheta = dtheta + ddtheta * dt;
    theta  = theta  + dtheta  * dt;
    dpsi   = dpsi   + ddpsi   * dt;
    psi    = psi    + dpsi    * dt;
    dz     = dz     + ddz     * dt;
    z      = z      + dz      * dt;
    
    % Penanganan kontak tanah (Ground Constraint)
    if z <= 0
        z = 0;
        if dz < 0
            dz = 0;
        end
        % Sudut-sudut ikut meredam jika wahana mendarat diam di tanah
        phi   = phi * 0.95;
        theta = theta * 0.95;
    end
    
    % Update State Vector untuk Iterasi Berikutnya
    x = [phi; dphi; theta; dtheta; psi; dpsi; z; dz];
end

fprintf('Simulasi selesai dalam %.2f detik CPU.\n', toc);

%% --- EKSPOR HASIL SIMULASI KE MAT & CSV ---------------------
sim_results = struct();
sim_results.time         = t_vec(:);
sim_results.phi_deg      = X(1,:)' * 180/pi;
sim_results.theta_deg    = X(3,:)' * 180/pi;
sim_results.psi_deg      = X(5,:)' * 180/pi;
sim_results.altitude_m   = X(7,:)';
sim_results.phi_est      = PHI_EST(:)   * 180/pi;
sim_results.theta_est    = THETA_EST(:) * 180/pi;
sim_results.throttle_pct = THR(:) * 100;                        % Dalam bentuk persentase
sim_results.servo_deg    = U;
sim_results.ref_roll     = REF(1,:)' * 180/pi;
sim_results.ref_pitch    = REF(2,:)' * 180/pi;
sim_results.ref_yaw      = REF(3,:)' * 180/pi;
sim_results.ref_alt      = REF(4,:)';
sim_results.gyro_x_logged = GYRO_X_LOG(:) * 180/pi;
sim_results.gyro_y_logged = GYRO_Y_LOG(:) * 180/pi;

% Simpan ke file MAT
save('hasil_simulasi_full.mat', 'sim_results');
fprintf('Hasil diekspor ke hasil_simulasi_full.mat (%d sampel).\n', N);

% Ekspor ke file CSV (kompatibilitas lintas-platform, misal Python)
T = table(sim_results.time, sim_results.phi_deg, sim_results.theta_deg, ...
    sim_results.psi_deg, sim_results.altitude_m, sim_results.phi_est, ...
    sim_results.theta_est, sim_results.throttle_pct, ...
    sim_results.ref_roll, sim_results.ref_pitch, sim_results.ref_yaw, sim_results.ref_alt, ...
    'VariableNames', {'time','phi_deg','theta_deg','psi_deg','altitude_m', ...
    'phi_est','theta_est','throttle_pct','ref_roll','ref_pitch','ref_yaw','ref_alt'});
writetable(T, 'hasil_simulasi_full.csv');
fprintf('Hasil diekspor ke hasil_simulasi_full.csv.\n');

%% ─── VISUALISASI DASHBOARD TERPADU ───────────────────────────
figure('Name','Dashboard Simulasi Full System Monocopter v4','Color','w','Position',[50 50 1400 850]);

% Subplot 1: Altitude Tracking (m)
subplot(3,2,1);
plot(t_vec, sim_results.ref_alt, 'r--', 'LineWidth', 1.5); hold on; grid on;
plot(t_vec, sim_results.altitude_m, 'b-', 'LineWidth', 2);
xlabel('Waktu (s)'); ylabel('Ketinggian z (m)');
title('Pelacakan Lintasan Ketinggian (Altitude Tracking)');
legend('Referensi', 'Aktual', 'Location', 'best');

% Subplot 2: Roll Tracking & Estimation (deg)
subplot(3,2,2);
plot(t_vec, sim_results.ref_roll, 'r--', 'LineWidth', 1.5); hold on; grid on;
plot(t_vec, sim_results.phi_deg, 'b-', 'LineWidth', 2);
plot(t_vec, sim_results.phi_est, 'g:', 'LineWidth', 1.5);
xlabel('Waktu (s)'); ylabel('Sikap Roll \phi (\circ)');
title('Pelacakan & Estimasi Kalman Sumbu Roll');
legend('Referensi', 'Aktual', 'Kalman Est', 'Location', 'best');

% Subplot 3: Pitch Tracking & Estimation (deg)
subplot(3,2,3);
plot(t_vec, sim_results.ref_pitch, 'r--', 'LineWidth', 1.5); hold on; grid on;
plot(t_vec, sim_results.theta_deg, 'b-', 'LineWidth', 2);
plot(t_vec, sim_results.theta_est, 'm:', 'LineWidth', 1.5);
xlabel('Waktu (s)'); ylabel('Sikap Pitch \theta (\circ)');
title('Pelacakan & Estimasi Kalman Sumbu Pitch');
legend('Referensi', 'Aktual', 'Kalman Est', 'Location', 'best');

% Subplot 4: Yaw Tracking (deg)
subplot(3,2,4);
plot(t_vec, sim_results.ref_yaw, 'r--', 'LineWidth', 1.5); hold on; grid on;
plot(t_vec, sim_results.psi_deg, 'b-', 'LineWidth', 2);
xlabel('Waktu (s)'); ylabel('Heading Yaw \psi (\circ)');
title('Pelacakan Sumbu Yaw (Heading)');
legend('Referensi', 'Aktual', 'Location', 'best');

% Subplot 5: Input Throttle Motor (%)
subplot(3,2,5);
plot(t_vec, sim_results.throttle_pct, 'k-', 'LineWidth', 1.8); grid on;
yline(throttle_hover*100, 'r:', 'Hover Level');
xlabel('Waktu (s)'); ylabel('Throttle (%)');
title('Sinyal Kendali Motor (Throttle)');
legend('Throttle Motor', 'Hover Nominal', 'Location', 'best');

% Subplot 6: Defleksi Servo S1-S4 (deg)
subplot(3,2,6);
plot(t_vec, sim_results.servo_deg(1,:), 'r-', 'LineWidth', 1.2); hold on; grid on;
plot(t_vec, sim_results.servo_deg(2,:), 'g-', 'LineWidth', 1.2);
plot(t_vec, sim_results.servo_deg(3,:), 'b-', 'LineWidth', 1.2);
plot(t_vec, sim_results.servo_deg(4,:), 'm-', 'LineWidth', 1.2);
yline(servo_max_deg, 'k--', 'Batas Maks');
yline(-servo_max_deg, 'k--', 'Batas Min');
xlabel('Waktu (s)'); ylabel('Defleksi Servo (\circ)');
title('Sinyal Keluaran Servo Aktuator');
legend('Servo 1 (Kiri)', 'Servo 2 (Depan)', 'Servo 3 (Kanan)', 'Servo 4 (Belakang)', 'Location', 'best');

sgtitle('Dashboard Simulasi Lengkap Closed-Loop Monocopter — Hukum Umpan-Balik-Keadaan (v4)', ...
    'FontSize', 15, 'FontWeight', 'bold');

% Fungsi Pembatas (Saturasi)
function y = clamp(x, lo, hi)
    y = max(lo, min(hi, x));
end