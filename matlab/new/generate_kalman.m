clear; clc; close all;

% Run monocopter_params to load base parameters dynamically
if exist('monocopter_params.m', 'file') == 2 || exist('monocopter_params', 'file') == 2
    monocopter_params;
end

%% =========================================================================
%% 1. KONFIGURASI MODEL DKF BAB III (DINAMIS DARI PARAMETER WAHANA)
%% =========================================================================
if exist('dt', 'var')
    dt_val = dt;
elseif exist('Ts_kf', 'var')
    dt_val = Ts_kf;
else
    dt_val = 0.005;
end

if exist('b_roll', 'var')
    b_roll_val = b_roll;
elseif exist('Ixx', 'var')
    b_roll_val = 1 / Ixx;
else
    b_roll_val = 201.215678;
end

if exist('Ad_kf', 'var')
    Ad_kf_val = Ad_kf;
else
    Ad_kf_val = [1, dt_val; 0, 1];
end

if exist('Bd_roll_kf', 'var')
    Bd_roll_kf_val = Bd_roll_kf;
else
    Bd_roll_kf_val = [0.5 * dt_val^2 * b_roll_val; dt_val * b_roll_val];
end

if exist('C_kf', 'var')
    C_kf_val = C_kf;
else
    C_kf_val = [1, 0];
end

fprintf('\n');
fprintf('============================================================\n');
fprintf('                 MODEL DKF BAB III\n');
fprintf('============================================================\n');
fprintf('Ad_kf =\n'); disp(Ad_kf_val);
fprintf('Bd_roll_kf =\n'); disp(Bd_roll_kf_val);
fprintf('C_kf =\n'); disp(C_kf_val);
fprintf('b_roll = %.6f\n', b_roll_val);
fprintf('dt     = %.6f s\n', dt_val);

%% =========================================================================
%% 2. PARAMETER KOVARIANSI BAB III
%% =========================================================================
if exist('Q_kf', 'var')
    Q_BAB3 = Q_kf;
    Q11_BAB3 = Q_kf(1,1);
    Q22_BAB3 = Q_kf(2,2);
else
    Q11_BAB3 = 1.0e-3;
    Q22_BAB3 = 3.0e-3;
    Q_BAB3 = diag([Q11_BAB3, Q22_BAB3]);
end

if exist('R_kf', 'var')
    R_BAB3 = R_kf;
else
    R_BAB3 = 3.0e-2;
end

fprintf('\n');
fprintf('------------------------------------------------------------\n');
fprintf('              PARAMETER DKF BAB III\n');
fprintf('------------------------------------------------------------\n');
fprintf('Q11 = %.6f\n', Q11_BAB3);
fprintf('Q22 = %.6f\n', Q22_BAB3);
fprintf('R   = %.6f\n', R_BAB3);
fprintf('\nQ BAB III =\n'); disp(Q_BAB3);

%% =========================================================================
%% 3. SINYAL UJI
%% =========================================================================
w_k = 2 * pi * 0.8;
t_k = 0:dt_val:10;
N_k = length(t_k);

% Ground truth attitude roll
phi_t = 0.2 * sin(w_k * t_k);

% Kecepatan sudut roll
dphi_t = 0.2 * w_k * cos(w_k * t_k);

% Percepatan sudut roll
ddphi_t = -0.2 * w_k^2 * sin(w_k * t_k);

% Torsi input berdasarkan model plant
tau_t = ddphi_t / b_roll_val;

%% =========================================================================
%% 4. SINYAL SENSOR AKSELEROMETER
%% =========================================================================
if exist('accel_noise_pwr', 'var')
    noise_std = sqrt(accel_noise_pwr / dt_val);
else
    noise_std = 0.141421;
end

rng(42);
accel_n = phi_t + noise_std * randn(1, N_k);

fprintf('\n');
fprintf('Standard deviation accelerometer = %.6f rad\n', noise_std);

%% =========================================================================
%% 5. VERIFIKASI BASELINE DKF BAB III
%% =========================================================================
[rmse_baseline, est_baseline, K_final, P_final] = ...
    run_dkf( ...
        Ad_kf_val, ...
        Bd_roll_kf_val, ...
        C_kf_val, ...
        Q_BAB3, ...
        R_BAB3, ...
        tau_t, ...
        accel_n, ...
        phi_t);

% RMSE raw sensor
rmse_raw = sqrt(mean((phi_t - accel_n).^2));

% Noise reduction terhadap raw measurement
noise_reduction = (rmse_raw - rmse_baseline) / rmse_raw * 100;

fprintf('\n');
fprintf('============================================================\n');
fprintf('              VERIFIKASI BASELINE DKF BAB III\n');
fprintf('============================================================\n');
fprintf('RMSE Raw     = %.6f rad (%.4f deg)\n', rmse_raw, rmse_raw*180/pi);
fprintf('RMSE Kalman  = %.6f rad (%.4f deg)\n', rmse_baseline, rmse_baseline*180/pi);
fprintf('Reduction    = %.1f %%\n', noise_reduction);
fprintf('\nQ baseline =\n'); disp(Q_BAB3);
fprintf('R baseline = %.6f\n', R_BAB3);
fprintf('Final Kalman Gain = %.6f\n', K_final);
fprintf('Final P Trace = %.6f\n', trace(P_final));
fprintf('============================================================\n');

%% =========================================================================
%% 6. KONFIGURASI GRID SEARCH 2D (OTOMATIS MEMUAT NILAI OPTIMAL)
%% =========================================================================
q_range_2d = unique(sort([1e-4, 1e-2, 1e-1, Q11_BAB3]));
r_range_2d = unique(sort([0.01, 0.10, 0.30, 0.50, R_BAB3]));

fprintf('\n');
fprintf('============================================================\n');
fprintf('                 GRID SEARCH 2D\n');
fprintf('============================================================\n');
fprintf('Q11 = '); fprintf('%.4g ', q_range_2d); fprintf('\n');
fprintf('R   = '); fprintf('%.2f ', r_range_2d); fprintf('\n');

fprintf('\nMenghitung Kalman Tuning Map 2D...\n');

% Hitung RMSE seluruh grid
RMSE_2D = compute_rmse_grid( ...
    q_range_2d, ...
    r_range_2d, ...
    Ad_kf_val, ...
    Bd_roll_kf_val, ...
    C_kf_val, ...
    tau_t, ...
    accel_n, ...
    phi_t);

%% =========================================================================
%% 7. CARI MINIMUM GRID HANYA UNTUK INFORMASI
%% =========================================================================
[min_rmse_grid, idx_min_grid] = min(RMSE_2D(:));
[q_idx_min, r_idx_min] = ind2sub(size(RMSE_2D), idx_min_grid);

q11_grid_min = q_range_2d(q_idx_min);
r_grid_min   = r_range_2d(r_idx_min);
q22_grid_min = 3 * q11_grid_min;

fprintf('\n');
fprintf('============================================================\n');
fprintf('           HASIL MINIMUM GRID SEARCH 2D\n');
fprintf('============================================================\n');
fprintf('Q11 minimum grid = %.6g\n', q11_grid_min);
fprintf('Q22 minimum grid = %.6g\n', q22_grid_min);
fprintf('R minimum grid   = %.6g\n', r_grid_min);
fprintf('RMSE minimum grid = %.6f rad\n', min_rmse_grid);
fprintf('RMSE minimum grid = %.4f deg\n', min_rmse_grid*180/pi);

fprintf('\nCATATAN:\n');
fprintf(['Nilai minimum grid di atas hanya digunakan sebagai analisis\n' ...
         'sensitivitas. Parameter final tetap mengikuti BAB III.\n']);

%% =========================================================================
%% 8. CARI POSISI KONFIGURASI BAB III PADA GRID
%% =========================================================================
q_idx_bab3 = find(q_range_2d == Q11_BAB3, 1);
r_idx_bab3 = find(r_range_2d == R_BAB3, 1);

if isempty(q_idx_bab3) || isempty(r_idx_bab3)
    error(['Parameter BAB III tidak ditemukan pada grid 2D. ' ...
           'Pastikan Q11 dan R terdapat pada grid.']);
end

% Ambil nilai RMSE dari grid
rmse_bab3_grid = RMSE_2D(q_idx_bab3, r_idx_bab3);

fprintf('\n');
fprintf('============================================================\n');
fprintf('             KONFIGURASI BAB III PADA GRID\n');
fprintf('============================================================\n');
fprintf('Q11 = %.6f\n', Q11_BAB3);
fprintf('Q22 = %.6f\n', Q22_BAB3);
fprintf('R   = %.6f\n', R_BAB3);
fprintf('RMSE Grid = %.6f rad\n', rmse_bab3_grid);
fprintf('RMSE Grid = %.4f deg\n', rmse_bab3_grid*180/pi);
fprintf('RMSE Direct DKF = %.6f rad\n', rmse_baseline);

%% =========================================================================
%% 9. TAMPILKAN MATRIKS RMSE 2D
%% =========================================================================
fprintf('\n');
fprintf('============================================================\n');
fprintf('          MATRIKS RMSE HASIL GRID SEARCH\n');
fprintf('============================================================\n');
fprintf('\n');
fprintf('%12s', 'Q11/R');
for j = 1:length(r_range_2d)
    fprintf('%12.2f', r_range_2d(j));
end
fprintf('\n');
fprintf('%s\n', repmat('-', 1, 72));

for i = 1:length(q_range_2d)
    fprintf('%12.1e', q_range_2d(i));
    for j = 1:length(r_range_2d)
        fprintf('%12.4f', RMSE_2D(i,j));
    end
    fprintf('\n');
end

%% =========================================================================
%% 10. KONFIGURASI VISUAL
%% =========================================================================
set(groot, 'DefaultAxesFontName',   'Times New Roman');
set(groot, 'DefaultTextFontName',   'Times New Roman');
set(groot, 'DefaultAxesFontSize',   11);
set(groot, 'DefaultTextFontSize',   11);
set(groot, 'DefaultAxesLineWidth',  1.0);
set(groot, 'DefaultLineLineWidth',  1.5);
set(groot, 'DefaultAxesBox',        'on');
set(groot, 'DefaultFigureColor',    'w');

% Folder output
outDir = 'figures_output';
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

%% =========================================================================
%% 11. GAMBAR 4.3.a - HEATMAP KALMAN TUNING MAP
%% =========================================================================
fig2d = figure( ...
    'Name', 'Gambar 4.3.a - Kalman Tuning Map', ...
    'Color', 'w', ...
    'Position', [100 100 1150 850]);

% Heatmap
imagesc(RMSE_2D);
colormap(parula);
hold on;

% Colorbar
cb = colorbar;
cb.Label.String = 'RMSE Sudut Roll (rad)';
cb.Label.FontName = 'Times New Roman';
cb.Label.FontSize = 11;
cb.FontName = 'Times New Roman';
cb.FontSize = 10;

% Ticks
r_labels = cellfun(@(v) sprintf('%.2f', v), num2cell(r_range_2d), 'UniformOutput', false);
q_labels = cellfun(@(v) sprintf('10^{%.0f}', log10(v)), num2cell(q_range_2d), 'UniformOutput', false);

set(gca, ...
    'XTick', 1:length(r_range_2d), ...
    'XTickLabel', r_labels, ...
    'YTick', 1:length(q_range_2d), ...
    'YTickLabel', q_labels, ...
    'YDir', 'normal', ...
    'FontName', 'Times New Roman', ...
    'FontSize', 10);

xlabel('Kovariansi Derau Pengukuran Sensor, R (rad^2)', 'FontName', 'Times New Roman', 'FontSize', 11);
ylabel('Kovariansi Derau Proses, Q_{11} (rad^2)', 'FontName', 'Times New Roman', 'FontSize', 11);

% Grid
ax = gca;
ax.XGrid = 'on'; ax.YGrid = 'on';
ax.GridColor = [1 1 1]; ax.GridAlpha = 0.65;
ax.LineWidth = 0.8;

% Text values in cells
for i = 1:length(q_range_2d)
    for j = 1:length(r_range_2d)
        value = RMSE_2D(i,j);
        if value > 0.075
            textColor = 'k';
        else
            textColor = 'w';
        end
        text(j, i, sprintf('%.4f', value), ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'middle', ...
            'FontName', 'Times New Roman', ...
            'FontSize', 10, ...
            'Color', textColor);
    end
end

%% =========================================================================
%% 12. MARKER KONFIGURASI BAB III
%% =========================================================================
rectangle('Position', [r_idx_bab3-0.5, q_idx_bab3-0.5, 1, 1], ...
    'EdgeColor', 'r', 'LineWidth', 3.0);

text(r_idx_bab3, q_idx_bab3 - 0.27, ...
    sprintf('Titik Optimal\nQ_{11}=%.1g, R=%.2g\nRMSE=%.4f rad', Q11_BAB3, R_BAB3, rmse_baseline), ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle', ...
    'FontName', 'Times New Roman', ...
    'FontSize', 9.0, ...
    'FontWeight', 'bold', ...
    'Color', 'r');

clim([min(RMSE_2D(:)), max(RMSE_2D(:))]);
box off;
hold off;

exportgraphics(fig2d, fullfile(outDir, 'gambar_4.3.a_kalman_tuning_map.png'), 'Resolution', 300);

%% =========================================================================
%% 13. GRID SURFACE 3D
%% =========================================================================
fprintf('\nMenghitung surface 3D...\n');

q_base_3d = logspace(-4, -1, 40);
r_base_3d = logspace(-2, 0, 40);

q_range_3d = unique(sort([q_base_3d, Q11_BAB3]));
r_range_3d = unique(sort([r_base_3d, R_BAB3]));

fprintf('Grid surface = %d x %d\n', length(q_range_3d), length(r_range_3d));

RMSE_3D = compute_rmse_grid( ...
    q_range_3d, ...
    r_range_3d, ...
    Ad_kf_val, ...
    Bd_roll_kf_val, ...
    C_kf_val, ...
    tau_t, ...
    accel_n, ...
    phi_t);

q_idx_3d_bab3 = find(q_range_3d == Q11_BAB3, 1);
r_idx_3d_bab3 = find(r_range_3d == R_BAB3, 1);

rmse_surface_bab3 = RMSE_3D(q_idx_3d_bab3, r_idx_3d_bab3);

fprintf('\n');
fprintf('============================================================\n');
fprintf('              SURFACE 3D - KONFIGURASI BAB III\n');
fprintf('============================================================\n');
fprintf('Q11 = %.6f\n', Q11_BAB3);
fprintf('Q22 = %.6f\n', Q22_BAB3);
fprintf('R   = %.6f\n', R_BAB3);
fprintf('RMSE Surface = %.6f rad\n', rmse_surface_bab3);
fprintf('RMSE Surface = %.4f deg\n', rmse_surface_bab3*180/pi);

%% =========================================================================
%% 14. GAMBAR 4.3.b - SURFACE 3D KALMAN TUNING MAP
%% =========================================================================
fig3d = figure( ...
    'Name', 'Gambar 4.3.b - Kalman Tuning Map Surface 3D', ...
    'Color', 'w', ...
    'Position', [100 80 1200 850]);

[Q_mesh, R_mesh] = meshgrid(q_range_3d, r_range_3d);

surf(Q_mesh, R_mesh, RMSE_3D', 'EdgeColor', 'none', 'FaceAlpha', 0.94);
colormap(parula);
shading interp;
hold on;

contour3(Q_mesh, R_mesh, RMSE_3D', 15, 'LineColor', [0.15 0.15 0.15], 'LineWidth', 0.65);

%% =========================================================================
%% 15. MARKER KONFIGURASI BAB III PADA SURFACE
%% =========================================================================
plot3(Q11_BAB3, R_BAB3, rmse_surface_bab3, 'o', ...
    'MarkerSize', 11, ...
    'LineWidth', 2.2, ...
    'MarkerEdgeColor', 'k', ...
    'MarkerFaceColor', [0.9290 0.6940 0.1250]);

%% =========================================================================
%% 16. LABEL KONFIGURASI BAB III
%% =========================================================================
text(Q11_BAB3, R_BAB3 * 1.7, rmse_surface_bab3 + 0.025, ...
    sprintf('Konfigurasi BAB III\nQ_{11}=%.1g\nR=%.2g\nRMSE=%.4f rad', Q11_BAB3, R_BAB3, rmse_baseline), ...
    'FontName', 'Times New Roman', ...
    'FontSize', 10, ...
    'FontWeight', 'bold', ...
    'Color', 'k', ...
    'BackgroundColor', [1 1 1 0.78], ...
    'Margin', 6);

set(gca, 'XScale', 'log', 'YScale', 'log');
xlabel('Kovariansi Derau Proses, Q_{11} (rad^2)', 'FontName', 'Times New Roman', 'FontSize', 11);
ylabel('Kovariansi Derau Pengukuran Sensor, R (rad^2)', 'FontName', 'Times New Roman', 'FontSize', 11);
zlabel('Kesalahan Sudut Estimasi, RMSE (rad)', 'FontName', 'Times New Roman', 'FontSize', 11);

cb3 = colorbar;
cb3.Label.String = 'RMSE (rad)';
cb3.Label.FontName = 'Times New Roman';
cb3.Label.FontSize = 11;
cb3.FontName = 'Times New Roman';
cb3.FontSize = 10;

view(-45, 28);
grid on;
ax3 = gca;
ax3.GridAlpha = 0.25;
ax3.XMinorGrid = 'on'; ax3.YMinorGrid = 'on';
ax3.Box = 'on';

exportgraphics(fig3d, fullfile(outDir, 'gambar_4.3.b_kalman_tuning_map.png'), 'Resolution', 300);
hold off;

%% =========================================================================
%% 20. RINGKASAN HASIL AKHIR
%% =========================================================================
fprintf('\n');
fprintf('============================================================\n');
fprintf('                    RINGKASAN AKHIR\n');
fprintf('============================================================\n');
fprintf('\nKONFIGURASI DKF BAB III / FINAL PENELITIAN:\n');
fprintf('Q11  = %.6e\n', Q11_BAB3);
fprintf('Q22  = %.6e\n', Q22_BAB3);
fprintf('R    = %.6e\n', R_BAB3);
fprintf('RMSE = %.6f rad (%.4f deg)\n', rmse_baseline, rmse_baseline*180/pi);

fprintf('\nRAW SENSOR:\n');
fprintf('RMSE Raw = %.6f rad (%.4f deg)\n', rmse_raw, rmse_raw*180/pi);
fprintf('Noise Reduction = %.2f %%\n', noise_reduction);

fprintf('\nMINIMUM PADA GRID SEARCH:\n');
fprintf('Q11  = %.6e\n', q11_grid_min);
fprintf('Q22  = %.6e\n', q22_grid_min);
fprintf('R    = %.6e\n', r_grid_min);
fprintf('RMSE = %.6f rad (%.4f deg)\n', min_rmse_grid, min_rmse_grid*180/pi);

fprintf('\nCATATAN:\n');
fprintf(['Minimum grid search tidak digunakan untuk mengganti\n' ...
         'parameter DKF BAB III. Konfigurasi final tetap:\n' ...
         'Q11 = %.1g, Q22 = %.1g, R = %.2g.\n'], Q11_BAB3, Q22_BAB3, R_BAB3);

fprintf('\nGambar berhasil diekspor ke:\n');
fprintf('  %s/gambar_4.3.a_kalman_tuning_map.png\n', outDir);
fprintf('  %s/gambar_4.3.b_kalman_tuning_map.png\n', outDir);
fprintf('============================================================\n');
fprintf('                 SELESAI\n');
fprintf('============================================================\n');

%% =========================================================================
%% FUNCTION 1: MENJALANKAN DKF
%% =========================================================================
function [rmse, est, K_final, P_final] = run_dkf(Ad, Bd, C, Q, R, tau, measurement, truth)
    N = length(truth);
    xK = [0; 0];
    PK = eye(2);
    est = zeros(1, N);
    K_final = zeros(2,1);

    for k = 1:N
        xp = Ad * xK + Bd * tau(k);
        Pp = Ad * PK * Ad' + Q;

        innovation = measurement(k) - C * xp;
        S = C * Pp * C' + R;
        K = Pp * C' / S;

        xK = xp + K * innovation;
        I = eye(size(Pp));
        PK = (I - K*C) * Pp * (I - K*C)' + K * R * K';

        est(k) = xK(1);
        K_final = K;
    end
    rmse = sqrt(mean((truth - est).^2));
    P_final = PK;
end

%% =========================================================================
%% FUNCTION 2: GRID SEARCH RMSE
%% =========================================================================
function RMSE = compute_rmse_grid(q_vec, r_vec, Ad, Bd, C, tau, measurement, truth)
    nq = length(q_vec);
    nr = length(r_vec);
    RMSE = zeros(nq, nr);

    for qi = 1:nq
        Q11 = q_vec(qi);
        Q22 = 3 * Q11;
        Q = diag([Q11, Q22]);

        for ri = 1:nr
            R = r_vec(ri);
            [rmse, ~, ~, ~] = run_dkf(Ad, Bd, C, Q, R, tau, measurement, truth);
            RMSE(qi, ri) = rmse;
        end
    end
end
