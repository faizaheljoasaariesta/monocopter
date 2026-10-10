clear; clc; monocopter_params; close all;

%% =========================================================================
%% 1. PARAMETER DISCRETE KALMAN FILTER (DINAMIS DARI MONOCOPTER_PARAMS)
%% =========================================================================
% Ditarik secara dinamis dari workspace monocopter_params.m
if exist('Ts_kf', 'var')
    Ts = Ts_kf;
elseif exist('dt', 'var')
    Ts = dt;
else
    Ts = 0.005;
end

if exist('Ad_kf', 'var')
    Ad = Ad_kf;
else
    Ad = [1, Ts; 0, 1];
end

if exist('C_kf', 'var')
    Cd = C_kf;
else
    Cd = [1, 0];
end

if exist('Q_kf', 'var')
    Q = Q_kf;
else
    Q = [0.001, 0; 0, 0.003];
end

if exist('R_kf', 'var')
    R = R_kf;
else
    R = 0.03;
end

if exist('P0_kf', 'var')
    P = P0_kf;
else
    P = eye(2);
end

%% =========================================================================
%% 2. JUMLAH ITERASI
%% =========================================================================
N = 400;
N_ss = 10000; % Iterasi tambahan untuk memperoleh nilai steady-state eksak

%% =========================================================================
%% 3. PENYIMPANAN HISTORI
%% =========================================================================
K_hist = zeros(N_ss, 2);
P_hist = zeros(N_ss, 4); % P11, P12, P21, P22

%% =========================================================================
%% 4. REKURSI DISCRETE KALMAN FILTER (DARE)
%% =========================================================================
for k = 1:N_ss
    % Prediction covariance
    P_priori = Ad * P * Ad' + Q;
    
    % Kalman Gain
    K = P_priori * Cd' / (Cd * P_priori * Cd' + R);
    
    % Correction covariance
    P = (eye(2) - K * Cd) * P_priori;
    
    % Simpan Kalman Gain dan Kovarians
    K_hist(k,:) = K';
    P_hist(k,:) = P(:)';
end

%% =========================================================================
%% 5. ITERASI YANG DITAMPILKAN PADA TABEL
%% =========================================================================
k_table = [1 5 10 20 100 200 400];
time_table = k_table * Ts;

% Kalman Gain
K1_table = K_hist(k_table,1);
K2_table = K_hist(k_table,2);

% Kovarians
P11_table = P_hist(k_table,1);
P22_table = P_hist(k_table,4);

%% =========================================================================
%% 6. NILAI STEADY-STATE EKSAL
%% =========================================================================
K_inf = K_hist(end,:)';
P_inf = reshape(P_hist(end,:),2,2);

%% =========================================================================
%% 7. BUAT TABEL
%% =========================================================================
T = table( ...
    k_table', ...
    time_table', ...
    K1_table, ...
    K2_table, ...
    P11_table, ...
    P22_table, ...
    'VariableNames', { ...
    'Iterasi_k', ...
    'Waktu_s', ...
    'K1', ...
    'K2', ...
    'P11', ...
    'P22'});

%% =========================================================================
%% 8. TAMPILKAN TABEL HASIL
%% =========================================================================
disp(' ');
disp('======================================================================');
disp(' EVOLUSI KALMAN GAIN DAN KOVARIANS MENUJU STEADY-STATE (KANAL ROLL)');
disp('======================================================================');
disp(T);

%% =========================================================================
%% 9. TAMPILKAN NILAI STEADY-STATE
%% =========================================================================
fprintf('\n======================================================================\n');
fprintf(' STEADY-STATE KALMAN FILTER (SSKF)\n');
fprintf('======================================================================\n');
fprintf('K1_inf  = %.8f\n', K_inf(1));
fprintf('K2_inf  = %.8f\n', K_inf(2));

fprintf('\nP_inf =\n');
fprintf('[ %.8f   %.8f ]\n', P_inf(1,1), P_inf(1,2));
fprintf('[ %.8f   %.8f ]\n', P_inf(2,1), P_inf(2,2));

%% =========================================================================
%% 10. FORMAT TABEL SKRIPSI/LAPORAN
%% =========================================================================
fprintf('\n======================================================================\n');
fprintf(' FORMAT TABEL SKRIPSI / LAPORAN\n');
fprintf('======================================================================\n');
fprintf('\n%-10s %-12s %-12s %-12s %-12s %-12s\n', ...
    'Iterasi', 'Waktu(s)', 'K1', 'K2', 'P11', 'P22');
fprintf('-----------------------------------------------------------------------\n');
for i = 1:length(k_table)
    fprintf('%-10d %-12.3f %-12.4f %-12.4f %-12.5f %-12.5f\n', ...
        k_table(i), ...
        time_table(i), ...
        K1_table(i), ...
        K2_table(i), ...
        P11_table(i), ...
        P22_table(i));
end
fprintf('%-10s %-12s %-12.4f %-12.4f %-12.5f %-12.5f\n', ...
    'inf', '--', ...
    K_inf(1), ...
    K_inf(2), ...
    P_inf(1,1), ...
    P_inf(2,2));
