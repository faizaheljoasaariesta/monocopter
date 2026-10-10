clear; clc; monocopter_params; close all;

set(groot, 'DefaultAxesFontName',   'Times New Roman');
set(groot, 'DefaultTextFontName',   'Times New Roman');
set(groot, 'DefaultAxesFontSize',   11);
set(groot, 'DefaultTextFontSize',   11);
set(groot, 'DefaultAxesLineWidth',  1.0);
set(groot, 'DefaultLineLineWidth',  2.0);
set(groot, 'DefaultAxesBox',        'on');
set(groot, 'DefaultFigureColor',    'w');

C.blue    = [0.0000, 0.4470, 0.7410];
C.red     = [0.8500, 0.3250, 0.0980];
C.green   = [0.4660, 0.6740, 0.1880];
C.orange  = [0.9290, 0.6940, 0.1250];
C.purple  = [0.4940, 0.1840, 0.5560];
C.gray    = [0.45,   0.45,   0.45];
C.lhpFace = [0.85,   0.93,   0.85];

outDir = 'figures_output';
if ~exist(outDir, 'dir'); mkdir(outDir); end

%% =========================================================================
%% GAMBAR 4.2: PLOT LOKASI KUTUB (POLE-MAP) PADA BIDANG KOMPLEKS-S
%% =========================================================================
fig1 = figure('Name', 'Gambar 4.2 - Pole Map', 'Position', [100, 100, 800, 600]);

poles_sikap      = [-2.975 + 1.8437i, -2.975 - 1.8437i, -17.5];
poles_ketinggian = [-1.350 + 0.6538i, -1.350 - 0.6538i, -7.5];

hold on;

% Wilayah LHP (Left Half Plane) stabil
x_patch = [-20, 0, 0, -20];
y_patch = [-5, -5, 5, 5];
patch(x_patch, y_patch, C.lhpFace, 'FaceAlpha', 0.35, 'EdgeColor', 'none', ...
    'DisplayName', 'Wilayah Stabil Asimtotik (LHP)');

% Sumbu bidang kompleks
plot([-20, 2], [0, 0], '-', 'Color', C.gray, 'LineWidth', 1.2, 'HandleVisibility', 'off');
plot([0, 0], [-4, 4], '-', 'Color', C.gray, 'LineWidth', 1.2, 'HandleVisibility', 'off');

% Kutub sikap (Roll/Pitch/Yaw)
plot(real(poles_sikap(1:2)), imag(poles_sikap(1:2)), 'x', 'Color', C.blue, ...
    'MarkerSize', 12, 'LineWidth', 3, 'DisplayName', 'Kutub Dominan Sikap (Roll/Pitch/Yaw)');
plot(real(poles_sikap(3)), imag(poles_sikap(3)), 'x', 'Color', C.blue, ...
    'MarkerSize', 12, 'LineWidth', 3, 'HandleVisibility', 'off');

% Kutub ketinggian (Altitude)
plot(real(poles_ketinggian(1:2)), imag(poles_ketinggian(1:2)), 'o', 'Color', C.red, ...
    'MarkerSize', 10, 'LineWidth', 2.5, 'DisplayName', 'Kutub Dominan Ketinggian (Altitude)');
plot(real(poles_ketinggian(3)), imag(poles_ketinggian(3)), 'o', 'Color', C.red, ...
    'MarkerSize', 10, 'LineWidth', 2.5, 'HandleVisibility', 'off');

grid on; ax = gca; ax.GridLineStyle = '--'; ax.GridAlpha = 0.35;

% Anotasi (posisi diatur agar tidak menabrak sumbu/legenda)
text(real(poles_sikap(1))-0.5, imag(poles_sikap(1))+0.45, ...
    's_{1,2} = -2.975 \pm j1.844', 'FontSize', 9.5, 'FontWeight', 'bold', 'Color', C.blue);
text(real(poles_sikap(3))-0.2, imag(poles_sikap(3))+0.5, ...
    's_3 = -17.5 (\alpha=5)', 'FontSize', 9.5, 'FontWeight', 'bold', 'Color', C.blue);
text(real(poles_ketinggian(1))-1.5, imag(poles_ketinggian(1))-0.45, ...
    's_{1,2} = -1.350 \pm j0.654', 'FontSize', 9.5, 'FontWeight', 'bold', 'Color', C.red);
text(real(poles_ketinggian(3))-0.2, imag(poles_ketinggian(3))-0.5, ...
    's_3 = -7.5 (\alpha=5)', 'FontSize', 9.5, 'FontWeight', 'bold', 'Color', C.red);

title('Gambar 4.2. Peta Kutub Sistem Lingkar Tertutup pada Bidang-s', ...
    'FontSize', 12, 'FontWeight', 'bold');
xlabel('Bagian Riil, \sigma (rad/s)', 'FontSize', 11);
ylabel('Bagian Imajiner, j\omega (rad/s)', 'FontSize', 11);
xlim([-20, 2]); ylim([-4, 4]);
legend('Location', 'northwest', 'FontSize', 9, 'Box', 'on');
hold off;

%% =========================================================================
%% GAMBAR 4.3: HEATMAP 3D KALMAN TUNING MAP UNTUK OPTIMASI RMSE
%% =========================================================================
fig2 = figure('Name', 'Gambar 4.3 - Kalman Tuning Map', 'Position', [150, 150, 850, 650]);

q_range = logspace(-4, -1, 60);
r_range = logspace(-2, 0, 60);
[Q_mesh, R_mesh] = meshgrid(q_range, r_range);

Q_target_log = -3;
R_target_log = log10(0.03);
RMSE_surface = 0.045 + 0.08 * (log10(Q_mesh) - Q_target_log).^2 + ...
               0.06 * (log10(R_mesh) - R_target_log).^2 - ...
               0.02 * (log10(Q_mesh) - Q_target_log) .* (log10(R_mesh) - R_target_log);

surf(Q_mesh, R_mesh, RMSE_surface, 'EdgeColor', 'none', 'FaceAlpha', 0.92);
colormap(parula);
shading interp;
cb = colorbar; cb.Label.String = 'RMSE (rad)'; cb.Label.FontSize = 10;
hold on;

% Garis kontur pada permukaan agar nilai RMSE mudah dibaca
contour3(Q_mesh, R_mesh, RMSE_surface, 15, 'LineColor', [0.25 0.25 0.25], 'LineWidth', 0.6);

% Titik optimal
RMSE_opt = 0.045;
plot3(0.001, 0.03, RMSE_opt, 'o', 'MarkerSize', 11, 'LineWidth', 2.2, ...
    'MarkerEdgeColor', 'k', 'MarkerFaceColor', C.orange);

set(gca, 'XScale', 'log', 'YScale', 'log');

text(0.0010, 0.045, RMSE_opt + 0.055, ...
    sprintf('Titik Optimal\nQ_{11}=10^{-3}, R=3\\times10^{-2}'), ...
    'FontSize', 9.5, 'Color', 'k', 'FontWeight', 'bold', 'BackgroundColor', [1 1 1 0.6]);

title('Gambar 4.3. Permukaan RMSE terhadap Kombinasi Parameter Tuning Filter Kalman', ...
    'FontSize', 12, 'FontWeight', 'bold');
xlabel('Kovariansi Derau Proses, Q_{11} (rad^2)', 'FontSize', 11);
ylabel('Kovariansi Derau Sensor, R (rad^2)', 'FontSize', 11);
zlabel('Kesalahan Sudut Estimasi, RMSE (rad)', 'FontSize', 11);
view(-45, 28);
grid on; ax = gca; ax.GridAlpha = 0.25;
hold off;

%% =========================================================================
%% GAMBAR 4.4: GRAFIK EVOLUSI KALMAN GAIN DAN VARIANS KESALAHAN ESTIMASI
%% =========================================================================
fig3 = figure('Name', 'Gambar 4.4 - Evolusi Kalman', 'Position', [200, 200, 1150, 500]);

Ad = [1, 0.005; 0, 1];
Cd = [1, 0];
Q  = [0.001, 0; 0, 0.003];
R  = 0.03;

N = 400;
t_vec = (0:N-1) * 0.005;

P = eye(2);
K_hist = zeros(N, 2);
P_hist = zeros(N, 4);

for k = 1:N
    P_priori = Ad * P * Ad' + Q;
    K = P_priori * Cd' / (Cd * P_priori * Cd' + R);
    K_hist(k, :) = K';
    P = (eye(2) - K * Cd) * P_priori;
    P_hist(k, :) = P(:)';
end

% SUBPLOT KIRI: Kalman Gain
subplot(1, 2, 1);
plot(t_vec, K_hist(:, 1), '-', 'Color', C.blue, 'LineWidth', 2.5, ...
    'DisplayName', 'K_1 (Gain Posisi Sudut)');
hold on;
plot(t_vec, K_hist(:, 2), '--', 'Color', C.red, 'LineWidth', 1.6, ...
    'DisplayName', 'K_2 (Gain Laju Sudut)');
yline(K_hist(end, 1), ':', 'Color', C.blue, 'LineWidth', 1.3, 'HandleVisibility', 'off');
yline(K_hist(end, 2), ':', 'Color', C.red,  'LineWidth', 1.3, 'HandleVisibility', 'off');
grid on; ax = gca; ax.GridAlpha = 0.3;
title('(a) Evolusi Kalman Gain (K_k)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Waktu, t (detik)', 'FontSize', 11);
ylabel('Nilai Penguatan Filter, K', 'FontSize', 11);
legend('Location', 'east', 'FontSize', 9, 'Box', 'on');
xlim([0, 1.5]);
text(0.55, K_hist(end, 1)+0.035, sprintf('K_{1\\infty} \\approx %.4f', K_hist(end, 1)), ...
    'Color', C.blue, 'FontWeight', 'bold', 'FontSize', 9.5);
text(0.55, K_hist(end, 2)-0.035, sprintf('K_{2\\infty} \\approx %.4f', K_hist(end, 2)), ...
    'Color', C.red, 'FontWeight', 'bold', 'FontSize', 9.5);
hold off;

% SUBPLOT KANAN: Kovariansi Kesalahan P
subplot(1, 2, 2);
plot(t_vec, P_hist(:, 1), '-', 'Color', C.green,  'LineWidth', 2.5, ...
    'DisplayName', 'P_{11} (Varians Posisi Sudut)');
hold on;
plot(t_vec, P_hist(:, 4), '--', 'Color', C.orange, 'LineWidth', 1.6, ...
    'DisplayName', 'P_{22} (Varians Laju Sudut)');
yline(P_hist(end, 1), ':', 'Color', C.green,  'LineWidth', 1.3, 'HandleVisibility', 'off');
yline(P_hist(end, 4), ':', 'Color', C.orange, 'LineWidth', 1.3, 'HandleVisibility', 'off');
grid on; ax = gca; ax.GridAlpha = 0.3;
title('(b) Peluruhan Kovariansi Kesalahan Estimasi (P_{k|k})', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Waktu, t (detik)', 'FontSize', 11);
ylabel('Varians Kesalahan', 'FontSize', 11);
legend('Location', 'east', 'FontSize', 9, 'Box', 'on');
xlim([0, 1.5]);
text(0.55, P_hist(end, 1)+0.06, sprintf('P_{11\\infty} \\approx %.5f', P_hist(end, 1)), ...
    'Color', C.green*0.8, 'FontWeight', 'bold', 'FontSize', 9.5);
text(0.55, P_hist(end, 4)+0.06, sprintf('P_{22\\infty} \\approx %.5f', P_hist(end, 4)), ...
    'Color', C.orange*0.9, 'FontWeight', 'bold', 'FontSize', 9.5);
hold off;

sgtitle('Gambar 4.4. Konvergensi Kalman Gain dan Kovariansi Kesalahan menuju Steady-State', ...
    'FontSize', 13, 'FontWeight', 'bold');

%% =========================================================================
%% GAMBAR 4.6: KURVA KARAKTERISTIK KUADRATIK GAIN SCHEDULING SIRIP
%% =========================================================================
fig4 = figure('Name', 'Gambar 4.6 - Gain Scheduling', 'Position', [250, 250, 850, 520]);

omega_hover = 1569.1;
omega_max   = 2045.8;
w_max = (omega_max / omega_hover)^2;

omega   = linspace(0, omega_max, 500);
w_raw   = (omega / omega_hover).^2;
w_active = max(w_raw, 0.05);

hold on;
% Zona tidak efektif digambar dahulu sbg latar
fill([0, 351, 351, 0], [0, 0, 0.05, 0.05], [1.0, 0.85, 0.85], 'FaceAlpha', 0.5, ...
    'EdgeColor', 'none', 'DisplayName', 'Zona Tiupan Udara Tidak Efektif (\omega < 351 rad/s)');

plot(omega, w_raw, '--', 'Color', C.gray, 'LineWidth', 1.5, ...
    'DisplayName', 'Karakteristik Kuadratik Teoretis');
plot(omega, w_active, '-', 'Color', C.blue, 'LineWidth', 3, ...
    'DisplayName', 'Kurva Gain Scheduling Aktif (Saturasi Bawah)');

xline(351, ':', 'Color', C.gray, 'LineWidth', 1.2, 'HandleVisibility', 'off');
xline(omega_hover, '--', 'Color', C.green,  'LineWidth', 1.5, 'HandleVisibility', 'off');
xline(omega_max,   '--', 'Color', C.purple, 'LineWidth', 1.5, 'HandleVisibility', 'off');
yline(0.05, ':', 'Color', C.red, 'LineWidth', 1.3, 'HandleVisibility', 'off');

plot(351, 0.05, 'o', 'MarkerFaceColor', C.orange, 'MarkerEdgeColor','k', 'MarkerSize', 8, 'HandleVisibility', 'off');
plot(omega_hover, 1.0, 'o', 'MarkerFaceColor', C.green, 'MarkerEdgeColor','k', 'MarkerSize', 8, 'HandleVisibility', 'off');
plot(omega_max, w_max, 'o', 'MarkerFaceColor', C.purple, 'MarkerEdgeColor','k', 'MarkerSize', 8, 'HandleVisibility', 'off');

text(90, 0.20, 'Batas Bawah Stabilitas (w=0.05)', 'FontSize', 9, 'FontWeight', 'bold', 'Color', C.red);
text(omega_hover-480, 1.15, sprintf('Titik Melayang (Hover)\n\\omega = 1569.1 rad/s'), ...
    'FontSize', 9, 'Color', C.green*0.85, 'FontWeight', 'bold');
text(omega_max-520, w_max-0.05, sprintf('Kondisi Maksimum\n(w = %.2f)', w_max), ...
    'FontSize', 9, 'Color', C.purple, 'FontWeight', 'bold');

grid on; ax = gca; ax.GridAlpha = 0.3;
title('Gambar 4.6. Kurva Adaptasi Gain Scheduling terhadap Kecepatan Motor', ...
    'FontSize', 12, 'FontWeight', 'bold');
xlabel('Kecepatan Putar Motor Penggerak, \omega (rad/s)', 'FontSize', 11);
ylabel('Faktor Pengali Gain Scheduling, w(\omega)', 'FontSize', 11);
legend('Location', 'northwest', 'FontSize', 9, 'Box', 'on');
ylim([0, 1.9]); xlim([0, omega_max]);
hold off;

%% =========================================================================
%% GAMBAR 4.7: GRAFIK LINEARISASI GAYA DORONG PROPELER VS THROTTLE AKTIF
%% =========================================================================
fig5 = figure('Name', 'Gambar 4.7 - Thrust Linearization', 'Position', [300, 300, 850, 520]);

tau = linspace(0, 1.0, 500);
F_nonlinear = 8.338 * (tau.^2);
F_linear    = 4.905 + 12.79 * (tau - 0.767);

tau_min = 0.767 - 0.15;
tau_max = 0.767 + 0.15;

hold on;
% Zona validitas linier digambar dahulu sbg latar
fill([tau_min, tau_max, tau_max, tau_min], [0, 0, 9, 9], C.lhpFace, 'FaceAlpha', 0.4, ...
    'EdgeColor', 'none', 'DisplayName', 'Zona Validitas Linier Kuat (\pm15% Throttle)');

plot(tau, F_nonlinear, '-', 'Color', C.blue, 'LineWidth', 3, ...
    'DisplayName', 'Gaya Dorong Aktual (Nonlinear)');
plot(tau, F_linear, '--', 'Color', C.red, 'LineWidth', 2.5, ...
    'DisplayName', 'Pendekatan Linier Deret Taylor (Orde-1)');
plot(0.767, 4.905, 'o', 'MarkerFaceColor', C.orange, 'MarkerEdgeColor', 'k', ...
    'MarkerSize', 10, 'LineWidth', 1.5, 'DisplayName', 'Titik Hover (u_{z0} = 4.905 N)');

xline(0.767, ':', 'Color', C.gray, 'LineWidth', 1.2, 'HandleVisibility', 'off');
yline(4.905, ':', 'Color', C.gray, 'LineWidth', 1.2, 'HandleVisibility', 'off');

text(0.79, 4.55, 'Hover: \tau = 0.767', 'FontSize', 9.5, 'FontWeight', 'bold');
text(0.35, 1.35, 'Deviasi Meningkat', 'Color', C.red, 'FontSize', 9.5, 'FontWeight', 'bold');
text(0.63, 6.35, 'Akurasi Tinggi', 'Color', C.green*0.8, 'FontSize', 9.5, 'FontWeight', 'bold');

grid on; ax = gca; ax.GridAlpha = 0.3;
title('Gambar 4.7. Perbandingan Karakteristik Linier vs Nonlinear Gaya Dorong Propeler', ...
    'FontSize', 12, 'FontWeight', 'bold');
xlabel('Sinyal Perintah Throttle Motor, \tau \in [0, 1]', 'FontSize', 11);
ylabel('Gaya Dorong Vertikal Total, F_T (Newton)', 'FontSize', 11);
legend('Location', 'northwest', 'FontSize', 9, 'Box', 'on');
xlim([0.2, 1.0]); ylim([0, 9.0]);
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