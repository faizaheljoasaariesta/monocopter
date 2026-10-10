clear; clc; monocopter_params; close all;

clearvars C s L G Gm Pm pcl ii nm bb Kp_ Ki_ Kd_ axes_ver info Acl Bcl Ccl sys_cl;

%% =========================================================================
%% 1. MUAT SKENARIO & REFERENSI
%% =========================================================================
hasilFile = 'hasil_simulasi_full.mat';
if ~exist(hasilFile, 'file')
  error(['Berkas "%s" tidak ditemukan.\n' ...
         'Jalankan simulasi_full_system.m TERLEBIH DAHULU untuk ' ...
         'menghasilkan skenario/referensi simulasi, baru jalankan skrip ini.'], hasilFile);
end
Sres = load(hasilFile);
if ~isfield(Sres, 'sim_results')
  error('Struktur "sim_results" tidak ditemukan di dalam %s.', hasilFile);
end
Rbase = Sres.sim_results;

t_vec = Rbase.time(:)';
N     = numel(t_vec);

rollRef_vec  = deg2rad(Rbase.ref_roll(:)');
pitchRef_vec = deg2rad(Rbase.ref_pitch(:)');
yawRef_vec   = deg2rad(Rbase.ref_yaw(:)');
altRef_vec   = Rbase.ref_alt(:)';

fprintf('Skenario & referensi dimuat dari %s (%d sampel).\n', hasilFile, N);

%% =========================================================================
%% 2. KONFIGURASI GAYA VISUAL GLOBAL
%% =========================================================================
set(groot, 'DefaultAxesFontName',   'Times New Roman');
set(groot, 'DefaultTextFontName',   'Times New Roman');
set(groot, 'DefaultAxesFontSize',   11);
set(groot, 'DefaultTextFontSize',   11);
set(groot, 'DefaultAxesLineWidth',  1.0);
set(groot, 'DefaultLineLineWidth',  2.0);
set(groot, 'DefaultAxesBox',        'on');
set(groot, 'DefaultFigureColor',    'w');

Pal.blue   = [0.0000, 0.4470, 0.7410];
Pal.red    = [0.8500, 0.3250, 0.0980];
Pal.green  = [0.2000, 0.6000, 0.2000];
Pal.orange = [0.9290, 0.6940, 0.1250];
Pal.gray   = [0.45,   0.45,   0.45];

outDir = 'figures_output';
if ~exist(outDir, 'dir'); mkdir(outDir); end

%% =========================================================================
%% 3. BANGKITKAN NOISE TERKONTROL
%% =========================================================================
rng(seed_gx); noise_ax   = normrnd(0, sqrt(accel_noise_sim / dt), [1, N]);
rng(seed_gy); noise_ay   = normrnd(0, sqrt(accel_noise_sim / dt), [1, N]);
rng(seed_ar); noise_gyro = normrnd(0, sqrt(gyro_noise_sim  / dt), [1, N]);

%% =========================================================================
%% 4. KUMPULKAN PARAMETER KE DALAM SATU STRUCT (utk simulator)
%% =========================================================================
P.m = m; P.g = g; P.Ixx = Ixx; P.Iyy = Iyy; P.Izz = Izz;
P.Ks_hover = Ks_hover; P.Ky_hover = Ky_hover; P.Q_hover = Q_hover; P.yaw_ff = yaw_ff;
P.Kp_roll = Kp_roll;  P.Kd_roll = Kd_roll;  P.Ki_roll = Ki_roll;
P.Kp_pitch = Kp_pitch; P.Kd_pitch = Kd_pitch; P.Ki_pitch = Ki_pitch;
P.Kp_yaw = Kp_yaw;    P.Kd_yaw = Kd_yaw;    P.Ki_yaw = Ki_yaw;
P.Kp_alt = Kp_alt;    P.Kd_alt = Kd_alt;    P.Ki_alt = Ki_alt;
P.pid_roll_int_max  = pid_roll_int_max;  P.pid_pitch_int_max = pid_pitch_int_max;
P.pid_yaw_int_max   = pid_yaw_int_max;   P.pid_alt_int_max   = pid_alt_int_max;
P.pid_roll_torque_max  = pid_roll_torque_max;  P.pid_pitch_torque_max = pid_pitch_torque_max;
P.pid_yaw_torque_max   = pid_yaw_torque_max;   P.pid_alt_force_max    = pid_alt_force_max;
P.b_roll = b_roll; P.b_pitch = b_pitch; P.b_yaw = b_yaw; P.b_alt = b_alt;
P.kT = kT; P.omega_max = omega_max; P.omega_hover = omega_hover;
P.throttle_hover = throttle_hover; P.F_to_throttle = F_to_throttle;
P.b_aero = b_aero; P.c_q = c_q; P.servo_min = servo_min; P.servo_max = servo_max;
P.Ad_kf = Ad_kf; P.Bd_roll_kf = Bd_roll_kf; P.Bd_pitch_kf = Bd_pitch_kf;
P.C_kf = C_kf; P.Q_kf = Q_kf; P.R_kf = R_kf; P.dt = dt;

refs.roll = rollRef_vec; refs.pitch = pitchRef_vec;
refs.yaw  = yawRef_vec;  refs.alt   = altRef_vec;

noise.ax = noise_ax; noise.ay = noise_ay; noise.gyro = noise_gyro;

%% =========================================================================
%% 5. JALANKAN EKSPERIMEN ABLASI
%% =========================================================================
fprintf('Menjalankan simulasi ablasi (3 kondisi terkontrol)...\n');

% Hipotesis 1: Kompensasi Feedforward Yaw (Kondisi A vs B)
res_B    = run_monocopter_sim(true,  'dkf', N, refs, noise, P);   % Desain penuh
res_A    = run_monocopter_sim(false, 'dkf', N, refs, noise, P);   % Tanpa feedforward

% Hipotesis 2: Estimasi DKF vs Sensor Mentah (feedforward tetap aktif)
res_raw  = run_monocopter_sim(true,  'raw', N, refs, noise, P);   % Feedback sensor mentah

fprintf('Simulasi ablasi selesai.\n\n');

%% =========================================================================
%% 6. GAMBAR 4.13: HIPOTESIS 1 (EFEK FEEDFORWARD YAW)
%% =========================================================================
fig1 = figure('Name', 'Gambar 4.13 - Komparasi Respons Sudut Yaw', ...
  'Position', [100, 100, 950, 550]);
ax = axes(fig1); hold(ax, 'on'); grid(ax, 'on'); ax.GridAlpha = 0.3;

y_limits = [-45, 95];
fill([0, 4, 4, 0], [y_limits(1), y_limits(1), y_limits(2), y_limits(2)], [1.0, 1.0, 0.8], ...
  'FaceAlpha', 0.35, 'EdgeColor', 'none', 'DisplayName', 'Fase Take-off & Melayang Awal');
fill([21, 28, 28, 21], [y_limits(1), y_limits(1), y_limits(2), y_limits(2)], [0.88, 0.88, 0.88], ...
  'FaceAlpha', 0.5, 'EdgeColor', 'none', 'DisplayName', 'Fase Landing & Touchdown');

plot(t_vec, rad2deg(yawRef_vec),  '--', 'Color', Pal.red,   'LineWidth', 2, ...
  'DisplayName', 'Referensi Setpoint Sumbu Yaw');
plot(t_vec, rad2deg(res_A.yaw),   '-',  'Color', Pal.blue,  'LineWidth', 2, ...
  'DisplayName', 'Kondisi A: Tanpa Feedforward (\delta_{ff} = 0)');
plot(t_vec, rad2deg(res_B.yaw),   '-',  'Color', Pal.green, 'LineWidth', 2.5, ...
  'DisplayName', sprintf('Kondisi B: Desain Penuh (\\delta_{ff} = %.2f^\\circ)', rad2deg(yaw_ff)));

title('Gambar 4.13. Efek Kompensasi Feedforward terhadap Torsi Reaksi Sumbu Yaw', ...
  'FontSize', 12, 'FontWeight', 'bold');
xlabel('Waktu (detik)', 'FontSize', 11);
ylabel('Sudut Yaw, \psi (derajat)', 'FontSize', 11);
xlim([0, t_vec(end)]); ylim(y_limits);
legend('Location', 'northwest', 'FontSize', 9, 'Box', 'on');

annotation(fig1, 'textarrow', [0.24, 0.28], [0.32, 0.40], ...
    'String', 'Kondisi A: Yaw hanyut akibat torsi reaksi', 'Color', Pal.blue, 'FontWeight', 'bold', 'FontSize', 9);
annotation(fig1, 'textarrow', [0.38, 0.34], [0.62, 0.56], ...
    'String', 'Kondisi B: stabil terkunci di 0^\circ', 'Color', Pal.green, 'FontWeight', 'bold', 'FontSize', 9);
hold(ax, 'off');

%% =========================================================================
%% 7. GAMBAR 4.14: HIPOTESIS 2 (DKF vs SENSOR MENTAH)
%% =========================================================================
fig2 = figure( ...
  'Name', 'Gambar 4.14 - Komparasi Fusi Sensor dan Respon Sikap', ...
  'Position', [150, 150, 950, 700]);

% (a) SUDUT ROLL

ax1 = subplot(2, 1, 1);

plot(t_vec, rad2deg(rollRef_vec), '--', ...
  'Color', Pal.red, ...
  'LineWidth', 1.8);
hold on;

plot(t_vec, rad2deg(res_raw.phi), '-', ...
  'Color', Pal.blue, ...
  'LineWidth', 1.3);

plot(t_vec, rad2deg(res_B.phi), '-', ...
  'Color', Pal.green, ...
  'LineWidth', 2.2);

grid on;

ax1.GridAlpha = 0.3;

ylabel('Sudut Roll, \phi (derajat)', ...
  'FontSize', 11);

ylim([-5, 18]);

% RENTANG WAKTU DISAMAKAN DENGAN GRAFIK BAWAH
xlim([0, t_vec(end)]);

title( ...
  'Gambar 4.14. Evaluasi Keandalan Umpan-Balik: Filter DKF vs Sensor Mentah', ...
  'FontSize', 12, ...
  'FontWeight', 'bold');

legend( ...
  'Referensi Setpoint', ...
  'Feedback Sensor Mentah (Tanpa DKF)', ...
  'Feedback Estimasi DKF', ...
  'Location', 'northeast', ...
  'FontSize', 8.5, ...
  'Box', 'on');

hold off;


% (b) DEFLEKSI SIRIP KEMUDI (SERVO 1)

ax2 = subplot(2, 1, 2);

plot(t_vec, rad2deg(res_raw.servo1), '-', ...
  'Color', Pal.blue, ...
  'LineWidth', 1.0);
hold on;

plot(t_vec, rad2deg(res_B.servo1), '-', ...
  'Color', Pal.green, ...
  'LineWidth', 2.2);

grid on;

ax2.GridAlpha = 0.3;

ylabel('Defleksi Sirip, \delta_1 (derajat)', ...
  'FontSize', 11);

xlabel('Waktu (detik)', ...
  'FontSize', 11);

ylim([-35, 35]);

% Rentang waktu sama dengan grafik atas
xlim([0, t_vec(end)]);

legend( ...
  'Defleksi dengan Feedback Sensor Mentah (jitter)', ...
  'Defleksi dengan Feedback DKF (mulus)', ...
  'Location', 'northeast', ...
  'FontSize', 8.5, ...
  'Box', 'on');

hold off;

%% =========================================================================
%% 8. EKSPOR OTOMATIS SEMUA GAMBAR
%% =========================================================================
figs     = [fig1, fig2];
figNames = {'Gambar_4_13_HipotesisFeedforwardYaw', 'Gambar_4_14_HipotesisDKFvsMentah'};

for i = 1:numel(figs)
  pngPath = fullfile(outDir, [figNames{i} '.png']);
  pdfPath = fullfile(outDir, [figNames{i} '.pdf']);
  exportgraphics(figs(i), pngPath, 'Resolution', 600);
  exportgraphics(figs(i), pdfPath, 'ContentType', 'vector');
  fprintf('Tersimpan: %s (.png & .pdf)\n', figNames{i});
end
fprintf('\nSemua gambar telah diekspor ke folder "%s/".\n', outDir);

%% =========================================================================
%% FUNGSI SIMULATOR UTAMA (INTERNAL FUNCTION)
%% =========================================================================
function res = run_monocopter_sim(enable_ff, feedback_mode, N, refs, noise, P)

  x = zeros(8, 1);
  int_r = 0; int_p = 0; int_y = 0; int_a = 0;

  xKr = [0; 0]; PKr = eye(2);
  xKp = [0; 0]; PKp = eye(2);

  res.phi    = zeros(1, N);
  res.yaw    = zeros(1, N);
  res.servo1 = zeros(1, N);

  tau_roll_prev  = 0.0;
  tau_pitch_prev = 0.0;

  if enable_ff
      yaw_ff_use = P.yaw_ff;
  else
      yaw_ff_use = 0.0;
  end

  for k = 1:N
      phi = x(1); dphi = x(2);
      theta = x(3); dtheta = x(4);
      psi = x(5); dpsi = x(6);
      z = max(0.0, x(7)); dz = x(8);

      phi_meas   = phi   + noise.ax(k);
      theta_meas = theta + noise.ay(k);
      dphi_meas  = dphi  + noise.gyro(k);
      dtheta_meas = dtheta + noise.gyro(k);

      % Filter Kalman (Roll)
      xp_r = P.Ad_kf * xKr + P.Bd_roll_kf * tau_roll_prev;
      Pp_r = P.Ad_kf * PKr * P.Ad_kf' + P.Q_kf;
      S_r  = P.C_kf * Pp_r * P.C_kf' + P.R_kf;
      K_r  = Pp_r * P.C_kf' / S_r;
      xKr  = xp_r + K_r * (phi_meas - P.C_kf * xp_r);
      PKr  = (eye(2) - K_r * P.C_kf) * Pp_r;
      phi_est = xKr(1); dphi_est = xKr(2);

      % Filter Kalman (Pitch)
      xp_p = P.Ad_kf * xKp + P.Bd_pitch_kf * tau_pitch_prev;
      Pp_p = P.Ad_kf * PKp * P.Ad_kf' + P.Q_kf;
      S_p  = P.C_kf * Pp_p * P.C_kf' + P.R_kf;
      K_p  = Pp_p * P.C_kf' / S_p;
      xKp  = xp_p + K_p * (theta_meas - P.C_kf * xp_p);
      PKp  = (eye(2) - K_p * P.C_kf) * Pp_p;
      theta_est = xKp(1); dtheta_est = xKp(2);

      % Pilihan sinyal umpan-balik
      if strcmp(feedback_mode, 'dkf')
          phi_fb = phi_est;   dphi_fb = dphi_est;
          theta_fb = theta_est; dtheta_fb = dtheta_est;
      else
          phi_fb = phi_meas;   dphi_fb = dphi_meas;
          theta_fb = theta_meas; dtheta_fb = dtheta_meas;
      end

      res.phi(k) = phi;
      res.yaw(k) = psi;

      % Integrator PID
      err_r = refs.roll(k)  - phi_fb;
      err_p = refs.pitch(k) - theta_fb;
      err_y = refs.yaw(k)   - psi;
      err_a = refs.alt(k)   - z;

      int_r = max(-P.pid_roll_int_max,  min(P.pid_roll_int_max,  int_r + err_r * P.dt));
      int_p = max(-P.pid_pitch_int_max, min(P.pid_pitch_int_max, int_p + err_p * P.dt));
      int_y = max(-P.pid_yaw_int_max,   min(P.pid_yaw_int_max,   int_y + err_y * P.dt));
      int_a = max(-P.pid_alt_int_max,   min(P.pid_alt_int_max,   int_a + err_a * P.dt));

      % Hukum kontrol umpan-balik-keadaan
      tau_roll  = -P.Kp_roll  * phi_fb   - P.Kd_roll  * dphi_fb   + P.Ki_roll  * int_r;
      tau_pitch = -P.Kp_pitch * theta_fb - P.Kd_pitch * dtheta_fb + P.Ki_pitch * int_p;
      tau_yaw   = -P.Kp_yaw   * psi      - P.Kd_yaw   * dpsi      + P.Ki_yaw   * int_y;
      u_z       = -P.Kp_alt   * z        - P.Kd_alt   * dz        + P.Ki_alt   * int_a;

      tau_roll  = max(-P.pid_roll_torque_max,  min(P.pid_roll_torque_max,  tau_roll));
      tau_pitch = max(-P.pid_pitch_torque_max, min(P.pid_pitch_torque_max, tau_pitch));
      tau_yaw   = max(-P.pid_yaw_torque_max,   min(P.pid_yaw_torque_max,   tau_yaw));
      u_z       = max(-P.pid_alt_force_max,    min(P.pid_alt_force_max,    u_z));

      tau_roll_prev  = tau_roll;
      tau_pitch_prev = tau_pitch;

      % Konversi gaya altitude -> throttle
      delta_tau = u_z * P.F_to_throttle;
      throttle_val = max(0.0, min(1.0, P.throttle_hover + delta_tau));

      omega_motor = throttle_val * P.omega_max;
      w_sch = max((omega_motor / P.omega_hover)^2, 0.05);

      tau_roll_c  = tau_roll  / w_sch;
      tau_pitch_c = tau_pitch / w_sch;
      tau_yaw_c   = tau_yaw   / w_sch;

      delta_phi   = tau_roll_c  / (2.0 * P.Ks_hover);
      delta_theta = tau_pitch_c / (2.0 * P.Ks_hover);
      delta_psi   = tau_yaw_c   / (4.0 * P.Ky_hover);

      d1 = delta_phi   + delta_psi + yaw_ff_use;
      d2 = delta_theta + delta_psi + yaw_ff_use;
      d3 = -delta_phi  + delta_psi + yaw_ff_use;
      d4 = -delta_theta + delta_psi + yaw_ff_use;

      d1 = max(P.servo_min, min(P.servo_max, d1));
      d2 = max(P.servo_min, min(P.servo_max, d2));
      d3 = max(P.servo_min, min(P.servo_max, d3));
      d4 = max(P.servo_min, min(P.servo_max, d4));

      res.servo1(k) = d1;

      % Dinamika fisik wahana
      F_T = P.kT * (omega_motor^2);
      Q_reaksi = P.c_q * F_T;

      tau_phi_act   = P.Ks_hover * w_sch * (d1 - d3) - P.b_aero * dphi;
      tau_theta_act = P.Ks_hover * w_sch * (d2 - d4) - P.b_aero * dtheta;
      tau_yaw_act   = P.Ky_hover * w_sch * (d1 + d2 + d3 + d4) - Q_reaksi - P.b_aero * dpsi;

      d2phi   = tau_phi_act   / P.Ixx;
      d2theta = tau_theta_act / P.Iyy;
      d2psi   = tau_yaw_act   / P.Izz;
      d2z     = -P.g + cos(phi) * cos(theta) * F_T / P.m;

      x(1) = x(1) + dphi   * P.dt;
      x(2) = x(2) + d2phi  * P.dt;
      x(3) = x(3) + dtheta * P.dt;
      x(4) = x(4) + d2theta * P.dt;
      x(5) = x(5) + dpsi   * P.dt;
      x(6) = x(6) + d2psi  * P.dt;

      if x(7) <= 0.0 && dz < 0
          x(7) = 0.0;
          x(8) = max(0.0, dz + d2z * P.dt);
          x(1) = x(1) * 0.95;
          x(3) = x(3) * 0.95;
      else
          x(7) = x(7) + dz * P.dt;
          x(8) = x(8) + d2z * P.dt;
      end
  end
end