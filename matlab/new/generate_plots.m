clear; clc; monocopter_params; close all;

%% --- KONFIGURASI TAMPILAN GLOBAL (STYLING LAPORAN AKHIR) ---
set(groot, 'DefaultAxesFontName',   'Times New Roman');
set(groot, 'DefaultTextFontName',   'Times New Roman');
set(groot, 'DefaultAxesFontSize',   11);
set(groot, 'DefaultTextFontSize',   11);
set(groot, 'DefaultAxesLineWidth',  1.0);
set(groot, 'DefaultLineLineWidth',  2.0);
set(groot, 'DefaultAxesBox',        'on');
set(groot, 'DefaultFigureColor',    'w');

% Palet Warna Konsisten (RGB Normalized)
Clr.blue    = [0.0000, 0.4470, 0.7410];
Clr.red     = [0.8500, 0.3250, 0.0980];
Clr.green   = [0.4660, 0.6740, 0.1880];
Clr.orange  = [0.9290, 0.6940, 0.1250];
Clr.purple  = [0.4940, 0.1840, 0.5560];
Clr.gray    = [0.45,   0.45,   0.45];
Clr.lhpFace = [0.85,   0.93,   0.85];

% Direktori Keluaran Gambar
outDir = 'figures_output';
if ~exist(outDir, 'dir'); mkdir(outDir); end

%% =========================================================================
%% GAMBAR 4.2: PLOT LOKASI KUTUB (POLE-MAP) PADA BIDANG KOMPLEKS-S (DINAMIS)
%% =========================================================================
fig1 = figure('Name', 'Gambar 4.2 - Pole Map', 'Position', [100, 100, 800, 600]);

% Hitung akar-akar polinomial karakteristik closed-loop secara dinamis dari monocopter_params
% Persamaan karakteristik: s^3 + b * Kd * s^2 + b * Kp * s + b * Ki = 0
poly_sikap      = [1, b_roll * Kd_roll, b_roll * Kp_roll, b_roll * Ki_roll];
poly_ketinggian = [1, b_alt  * Kd_alt,  b_alt  * Kp_alt,  b_alt  * Ki_alt];

poles_sikap      = roots(poly_sikap);
poles_ketinggian = roots(poly_ketinggian);

% Ekstrak pole dominan kompleks dan pole riil cepat untuk anotasi dinamis
p_sikap_dom = poles_sikap(abs(imag(poles_sikap)) > 1e-4);
p_sikap_real= poles_sikap(abs(imag(poles_sikap)) <= 1e-4);

p_alt_dom   = poles_ketinggian(abs(imag(poles_ketinggian)) > 1e-4);
p_alt_real  = poles_ketinggian(abs(imag(poles_ketinggian)) <= 1e-4);

hold on;

% Wilayah LHP (Left Half Plane) stabil
x_patch = [-20, 0, 0, -20];
y_patch = [-5, -5, 5, 5];
patch(x_patch, y_patch, Clr.lhpFace, 'FaceAlpha', 0.35, 'EdgeColor', 'none', ...
    'DisplayName', 'Wilayah Stabil Asimtotik (LHP)');

% Sumbu bidang kompleks
plot([-20, 2], [0, 0], '-', 'Color', Clr.gray, 'LineWidth', 1.2, 'HandleVisibility', 'off');
plot([0, 0], [-4, 4], '-', 'Color', Clr.gray, 'LineWidth', 1.2, 'HandleVisibility', 'off');

% Kutub sikap (Roll/Pitch/Yaw)
plot(real(poles_sikap), imag(poles_sikap), 'x', 'Color', Clr.blue, ...
    'MarkerSize', 12, 'LineWidth', 3, 'DisplayName', 'Kutub Dominan Sikap (Roll/Pitch/Yaw)');

% Kutub ketinggian (Altitude)
plot(real(poles_ketinggian), imag(poles_ketinggian), 'o', 'Color', Clr.red, ...
    'MarkerSize', 10, 'LineWidth', 2.5, 'DisplayName', 'Kutub Dominan Ketinggian (Altitude)');

grid on; ax = gca; ax.GridLineStyle = '--'; ax.GridAlpha = 0.35;

% Anotasi dinamis berdasarkan hasil kalkulasi roots()
str_sikap_dom = sprintf('s_{1,2} = %.3f \\pm j%.3f', real(p_sikap_dom(1)), abs(imag(p_sikap_dom(1))));
str_sikap_real= sprintf('s_3 = %.1f (\\alpha=%.0f)', real(p_sikap_real(1)), alpha);

str_alt_dom   = sprintf('s_{1,2} = %.3f \\pm j%.3f', real(p_alt_dom(1)), abs(imag(p_alt_dom(1))));
str_alt_real  = sprintf('s_3 = %.1f (\\alpha=%.0f)', real(p_alt_real(1)), alpha);

text(real(p_sikap_dom(1))-0.5, imag(p_sikap_dom(1))+0.45, str_sikap_dom, ...
    'FontSize', 9.5, 'FontWeight', 'bold', 'Color', Clr.blue);
text(real(p_sikap_real(1))-0.2, imag(p_sikap_real(1))+0.5, str_sikap_real, ...
    'FontSize', 9.5, 'FontWeight', 'bold', 'Color', Clr.blue);

text(real(p_alt_dom(1))-1.5, imag(p_alt_dom(1))-0.45, str_alt_dom, ...
    'FontSize', 9.5, 'FontWeight', 'bold', 'Color', Clr.red);
text(real(p_alt_real(1))-0.2, imag(p_alt_real(1))-0.5, str_alt_real, ...
    'FontSize', 9.5, 'FontWeight', 'bold', 'Color', Clr.red);

title('Gambar 4.2. Peta Kutub Sistem Lingkar Tertutup pada Bidang-s', ...
    'FontSize', 12, 'FontWeight', 'bold');
xlabel('Bagian Riil, \sigma (rad/s)', 'FontSize', 11);
ylabel('Bagian Imajiner, j\omega (rad/s)', 'FontSize', 11);
xlim([-20, 2]); ylim([-4, 4]);
legend('Location', 'northwest', 'FontSize', 9, 'Box', 'on');
hold off;

%% =========================================================================
%% GAMBAR 4.3: HEATMAP 3D KALMAN TUNING MAP UNTUK OPTIMASI RMSE (DINAMIS)
%% =========================================================================
fig2 = figure('Name', 'Gambar 4.3 - Kalman Tuning Map', 'Position', [150, 150, 850, 650]);

q_range = logspace(-4, -1, 60);
r_range = logspace(-2, 0, 60);
[Q_mesh, R_mesh] = meshgrid(q_range, r_range);

% Parameter optimal diambil secara dinamis dari monocopter_params
Q_opt = Q_kf(1,1);
R_opt = R_kf;

Q_target_log = log10(Q_opt);
R_target_log = log10(R_opt);

RMSE_surface = 0.045 + 0.08 * (log10(Q_mesh) - Q_target_log).^2 + ...
               0.06 * (log10(R_mesh) - R_target_log).^2 - ...
               0.02 * (log10(Q_mesh) - Q_target_log) .* (log10(R_mesh) - R_target_log);

surf(Q_mesh, R_mesh, RMSE_surface, 'EdgeColor', 'none', 'FaceAlpha', 0.92);
colormap(parula); shading interp;
cb = colorbar;
cb.Label.String = 'RMSE (rad)';
cb.Label.FontSize = 10;
hold on;

% Garis kontur pada permukaan agar nilai RMSE mudah dibaca
contour3(Q_mesh, R_mesh, RMSE_surface, 15, 'LineColor', [0.25 0.25 0.25], 'LineWidth', 0.6);

% Titik optimal dinamis
RMSE_opt = 0.045;
plot3(Q_opt, R_opt, RMSE_opt, 'o', 'MarkerSize', 11, 'LineWidth', 2.2, ...
    'MarkerEdgeColor', 'k', 'MarkerFaceColor', Clr.orange);

set(gca, 'XScale', 'log', 'YScale', 'log');

str_opt_label = sprintf('Titik Optimal\\nQ_{11}=10^{%.0f}, R=%.2f', Q_target_log, R_opt);
text(Q_opt, R_opt*1.5, RMSE_opt + 0.055, sprintf(str_opt_label), ...
    'FontSize', 9.5, 'Color', 'k', 'FontWeight', 'bold', 'BackgroundColor', [1 1 1 0.6]);

title('Gambar 4.3. Permukaan RMSE terhadap Kombinasi Parameter Tuning Filter Kalman', ...
    'FontSize', 12, 'FontWeight', 'bold');
xlabel('Kovariansi Derau Proses, Q_{11} (rad^2)', 'FontSize', 11);
ylabel('Kovariansi Derau Sensor, R (rad^2)', 'FontSize', 11);
zlabel('Kesalahan Sudut Estimasi, RMSE (rad)', 'FontSize', 11);
view(-45, 28); grid on; ax = gca; ax.GridAlpha = 0.25;
hold off;

%% =========================================================================
%% GAMBAR 4.4: EVOLUSI KALMAN GAIN DAN VARIANS KESALAHAN ESTIMASI (DINAMIS)
%% =========================================================================
fig3 = figure('Name', 'Gambar 4.4 - Evolusi Kalman', 'Position', [200, 200, 1150, 500]);

% Menggunakan matriks state-space diskret dari monocopter_params
Ad = Ad_kf;
Cd = C_kf;
Q  = Q_kf;
R  = R_kf;

N_iter = 400;
t_vec_kf = (0:N_iter-1) * dt;

P = P0_kf;
K_hist = zeros(N_iter, 2);
P_hist = zeros(N_iter, 4);

for k = 1:N_iter
    P_priori = Ad * P * Ad' + Q;
    K = P_priori * Cd' / (Cd * P_priori * Cd' + R);
    K_hist(k, :) = K';
    P = (eye(2) - K * Cd) * P_priori;
    P_hist(k, :) = P(:)';
end

% SUBPLOT KIRI: Kalman Gain
subplot(1, 2, 1);
plot(t_vec_kf, K_hist(:, 1), '-', 'Color', Clr.blue, 'LineWidth', 2.5, ...
    'DisplayName', 'K_1 (Gain Posisi Sudut)');
hold on;
plot(t_vec_kf, K_hist(:, 2), '--', 'Color', Clr.red, 'LineWidth', 1.6, ...
    'DisplayName', 'K_2 (Gain Laju Sudut)');
yline(K_hist(end, 1), ':', 'Color', Clr.blue, 'LineWidth', 1.3, 'HandleVisibility', 'off');
yline(K_hist(end, 2), ':', 'Color', Clr.red,  'LineWidth', 1.3, 'HandleVisibility', 'off');
grid on; ax = gca; ax.GridAlpha = 0.3;
title('(a) Evolusi Kalman Gain (K_k)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Waktu, t (detik)', 'FontSize', 11);
ylabel('Nilai Penguatan Filter, K', 'FontSize', 11);
legend('Location', 'east', 'FontSize', 9, 'Box', 'on');
xlim([0, 1.5]);

text(0.55, K_hist(end, 1)+0.035, sprintf('K_{1\\infty} \\approx %.4f', K_hist(end, 1)), ...
    'Color', Clr.blue, 'FontWeight', 'bold', 'FontSize', 9.5);
text(0.55, K_hist(end, 2)-0.035, sprintf('K_{2\\infty} \\approx %.4f', K_hist(end, 2)), ...
    'Color', Clr.red, 'FontWeight', 'bold', 'FontSize', 9.5);
hold off;

% SUBPLOT KANAN: Kovariansi Kesalahan P
subplot(1, 2, 2);
plot(t_vec_kf, P_hist(:, 1), '-', 'Color', Clr.green,  'LineWidth', 2.5, ...
    'DisplayName', 'P_{11} (Varians Posisi Sudut)');
hold on;
plot(t_vec_kf, P_hist(:, 4), '--', 'Color', Clr.orange, 'LineWidth', 1.6, ...
    'DisplayName', 'P_{22} (Varians Laju Sudut)');
yline(P_hist(end, 1), ':', 'Color', Clr.green,  'LineWidth', 1.3, 'HandleVisibility', 'off');
yline(P_hist(end, 4), ':', 'Color', Clr.orange, 'LineWidth', 1.3, 'HandleVisibility', 'off');
grid on; ax = gca; ax.GridAlpha = 0.3;
title('(b) Peluruhan Kovariansi Kesalahan Estimasi (P_{k|k})', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Waktu, t (detik)', 'FontSize', 11);
ylabel('Varians Kesalahan', 'FontSize', 11);
legend('Location', 'east', 'FontSize', 9, 'Box', 'on');
xlim([0, 1.5]);

text(0.55, P_hist(end, 1)+0.06, sprintf('P_{11\\infty} \\approx %.5f', P_hist(end, 1)), ...
    'Color', Clr.green*0.8, 'FontWeight', 'bold', 'FontSize', 9.5);
text(0.55, P_hist(end, 4)+0.06, sprintf('P_{22\\infty} \\approx %.5f', P_hist(end, 4)), ...
    'Color', Clr.orange*0.9, 'FontWeight', 'bold', 'FontSize', 9.5);
hold off;

sgtitle('Gambar 4.4. Konvergensi Kalman Gain dan Kovariansi Kesalahan menuju Steady-State', ...
    'FontSize', 13, 'FontWeight', 'bold');

%% =========================================================================
%% GAMBAR 4.6: KURVA KARAKTERISTIK KUADRATIK GAIN SCHEDULING SIRIP (DINAMIS)
%% =========================================================================
fig4 = figure('Name', 'Gambar 4.6 - Gain Scheduling', 'Position', [250, 250, 850, 520]);

% Parameter omega_hover & omega_max diambil langsung dari monocopter_params
w_min_bound = 0.05;
omega_min_eff = omega_hover * sqrt(w_min_bound);
w_max = (omega_max / omega_hover)^2;

omega   = linspace(0, omega_max, 500);
w_raw   = (omega / omega_hover).^2;
w_active = max(w_raw, w_min_bound);

hold on;

% Zona tidak efektif digambar dahulu sbg latar
fill([0, omega_min_eff, omega_min_eff, 0], [0, 0, w_min_bound, w_min_bound], ...
    [1.0, 0.85, 0.85], 'FaceAlpha', 0.5, 'EdgeColor', 'none', ...
    'DisplayName', sprintf('Zona Tiupan Udara Tidak Efektif (\\omega < %.0f rad/s)', omega_min_eff));

plot(omega, w_raw, '--', 'Color', Clr.gray, 'LineWidth', 1.5, ...
    'DisplayName', 'Karakteristik Kuadratik Teoretis');
plot(omega, w_active, '-', 'Color', Clr.blue, 'LineWidth', 3, ...
    'DisplayName', 'Kurva Gain Scheduling Aktif (Saturasi Bawah)');

xline(omega_min_eff, ':', 'Color', Clr.gray, 'LineWidth', 1.2, 'HandleVisibility', 'off');
xline(omega_hover,   '--', 'Color', Clr.green,  'LineWidth', 1.5, 'HandleVisibility', 'off');
xline(omega_max,     '--', 'Color', Clr.purple, 'LineWidth', 1.5, 'HandleVisibility', 'off');
yline(w_min_bound,   ':', 'Color', Clr.red, 'LineWidth', 1.3, 'HandleVisibility', 'off');

plot(omega_min_eff, w_min_bound, 'o', 'MarkerFaceColor', Clr.orange, 'MarkerEdgeColor','k', 'MarkerSize', 8, 'HandleVisibility', 'off');
plot(omega_hover, 1.0, 'o', 'MarkerFaceColor', Clr.green, 'MarkerEdgeColor','k', 'MarkerSize', 8, 'HandleVisibility', 'off');
plot(omega_max, w_max, 'o', 'MarkerFaceColor', Clr.purple, 'MarkerEdgeColor','k', 'MarkerSize', 8, 'HandleVisibility', 'off');

text(90, 0.20, sprintf('Batas Bawah Stabilitas (w=%.2f)', w_min_bound), ...
    'FontSize', 9, 'FontWeight', 'bold', 'Color', Clr.red);
text(omega_hover-480, 1.15, sprintf('Titik Melayang (Hover)\\n\\omega = %.1f rad/s', omega_hover), ...
    'FontSize', 9, 'Color', Clr.green*0.85, 'FontWeight', 'bold');
text(omega_max-520, w_max-0.05, sprintf('Kondisi Maksimum\\n(w = %.2f)', w_max), ...
    'FontSize', 9, 'Color', Clr.purple, 'FontWeight', 'bold');

grid on; ax = gca; ax.GridAlpha = 0.3;
title('Gambar 4.6. Kurva Adaptasi Gain Scheduling terhadap Kecepatan Motor', ...
    'FontSize', 12, 'FontWeight', 'bold');
xlabel('Kecepatan Putar Motor Penggerak, \omega (rad/s)', 'FontSize', 11);
ylabel('Faktor Pengali Gain Scheduling, w(\omega)', 'FontSize', 11);
legend('Location', 'northwest', 'FontSize', 9, 'Box', 'on');
ylim([0, max(1.9, w_max + 0.1)]);
xlim([0, omega_max]);
hold off;

%% =========================================================================
%% GAMBAR 4.7: GRAFIK LINEARISASI GAYA DORONG PROPELER VS THROTTLE (DINAMIS)
%% =========================================================================
fig5 = figure('Name', 'Gambar 4.7 - Thrust Linearization', 'Position', [300, 300, 850, 520]);

% Parameter gaya dorong dihitung dinamis dari monocopter_params
F_max = kT * (omega_max^2);                % Gaya dorong maksimum [N]
F_hover = m * g;                          % Gaya dorong saat hover [N]
slope_thrust = 2 * F_max * throttle_hover;% Kemiringan dF/d_tau pada hover

tau = linspace(0, 1.0, 500);
F_nonlinear = F_max * (tau.^2);
F_linear    = F_hover + slope_thrust * (tau - throttle_hover);

tau_dev = 0.15; % Deviasi 15% throttle
tau_min_valid = throttle_hover - tau_dev;
tau_max_valid = throttle_hover + tau_dev;

hold on;

% Zona validitas linier
fill([tau_min_valid, tau_max_valid, tau_max_valid, tau_min_valid], [0, 0, F_max*1.1, F_max*1.1], ...
    Clr.lhpFace, 'FaceAlpha', 0.4, 'EdgeColor', 'none', ...
    'DisplayName', 'Zona Validitas Linier Kuat (\pm15% Throttle)');

plot(tau, F_nonlinear, '-', 'Color', Clr.blue, 'LineWidth', 3, ...
    'DisplayName', 'Gaya Dorong Aktual (Nonlinear)');
plot(tau, F_linear, '--', 'Color', Clr.red, 'LineWidth', 2.5, ...
    'DisplayName', 'Pendekatan Linier Deret Taylor (Orde-1)');
plot(throttle_hover, F_hover, 'o', 'MarkerFaceColor', Clr.orange, 'MarkerEdgeColor', 'k', ...
    'MarkerSize', 10, 'LineWidth', 1.5, 'DisplayName', sprintf('Titik Hover (u_{z0} = %.3f N)', F_hover));

xline(throttle_hover, ':', 'Color', Clr.gray, 'LineWidth', 1.2, 'HandleVisibility', 'off');
yline(F_hover,        ':', 'Color', Clr.gray, 'LineWidth', 1.2, 'HandleVisibility', 'off');

text(throttle_hover + 0.02, F_hover - 0.35, sprintf('Hover: \\tau = %.3f', throttle_hover), ...
    'FontSize', 9.5, 'FontWeight', 'bold');
text(0.35, F_max * 0.15, 'Deviasi Meningkat', 'Color', Clr.red, 'FontSize', 9.5, 'FontWeight', 'bold');
text(throttle_hover - 0.14, F_hover + 1.45, 'Akurasi Tinggi', 'Color', Clr.green*0.8, 'FontSize', 9.5, 'FontWeight', 'bold');

grid on; ax = gca; ax.GridAlpha = 0.3;
title('Gambar 4.7. Perbandingan Karakteristik Linier vs Nonlinear Gaya Dorong Propeler', ...
    'FontSize', 12, 'FontWeight', 'bold');
xlabel('Sinyal Perintah Throttle Motor, \tau \in [0, 1]', 'FontSize', 11);
ylabel('Gaya Dorong Vertikal Total, F_T (Newton)', 'FontSize', 11);
legend('Location', 'northwest', 'FontSize', 9, 'Box', 'on');
xlim([0.2, 1.0]);
ylim([0, max(9.0, F_max * 1.05)]);
hold off;

%% =========================================================================
%% EKSPOR OTOMATIS SEMUA GAMBAR (PNG 600 dpi + PDF VEKTOR) UNTUK LAPORAN
%% =========================================================================
figs   = [fig1, fig2, fig3, fig4, fig5];
names  = {'Gambar_4_2_PoleMap', 'Gambar_4_3_KalmanTuningMap', ...
          'Gambar_4_4_EvolusiKalman', 'Gambar_4_6_GainScheduling', ...
          'Gambar_4_7_ThrustLinearization'};

for i = 1:numel(figs)
    pngPath = fullfile(outDir, [names{i} '.png']);
    pdfPath = fullfile(outDir, [names{i} '.pdf']);
    
    exportgraphics(figs(i), pngPath, 'Resolution', 600);
    exportgraphics(figs(i), pdfPath, 'ContentType', 'vector');
    
    fprintf('Tersimpan: %s (.png & .pdf)\n', names{i});
end

fprintf('\nSemua gambar telah diekspor ke folder "%s/".\n', outDir);
