%% --- KONFIGURASI: UBAH BARIS INI SESUAI UJI YANG DIJALANKAN ---
AXIS = 'roll';   % <-- pilih: 'roll' | 'pitch' | 'yaw' | 'alt'

%% --- Peta konfigurasi per sumbu (tidak perlu diubah) ---
switch AXIS
    case 'roll'
        script_var = 'roll_sr';   t_var = 't_sr';
        simulink_field = 'phi_slx';
        ref_val = 12;    unit = 'deg';  conv = 180/pi;
        axis_label = 'Roll \phi (\circ)';
        title_label = 'Roll';
        run_hint = 'step_response_roll';
    case 'pitch'
        script_var = 'pitch_sp';  t_var = 't_sp';
        simulink_field = 'theta_slx';
        ref_val = 10;    unit = 'deg';  conv = 180/pi;
        axis_label = 'Pitch \theta (\circ)';
        title_label = 'Pitch';
        run_hint = 'step_response_pitch';
    case 'yaw'
        script_var = 'yaw_sy';    t_var = 't_sy';
        simulink_field = 'psi_slx';
        ref_val = 30;    unit = 'deg';  conv = 180/pi;
        axis_label = 'Yaw \psi (\circ)';
        title_label = 'Yaw';
        run_hint = 'step_response_yaw_track';
    case 'alt'
        script_var = 'alt_sa';    t_var = 't_sa';
        simulink_field = 'z_slx';
        ref_val = 3.0;   unit = 'm';    conv = 1;
        axis_label = 'Altitude z (m)';
        title_label = 'Altitude';
        run_hint = 'step_response_alt';
    otherwise
        error('AXIS tidak dikenali: "%s". Pilih roll/pitch/yaw/alt.', AXIS);
end

%% --- Ambil sinyal Simulink ---
if ~exist('out','var')
    error(['Variabel out belum ada. Jalankan model Simulink (Run) dahulu. ' ...
           'Hasil tersimpan dalam objek out.']);
end
try
    sig = out.(simulink_field);
catch
    error(['out.%s tidak ditemukan. Pastikan blok To Workspace pada ' ...
           'sinyal terkait: Variable name = %s, Save format = ' ...
           'Timeseries. Lihat daftar blok yang dibutuhkan di komentar ' ...
           'awal skrip ini.'], simulink_field, simulink_field);
end

%% --- Ambil data script ---
if ~exist(script_var,'var') || ~exist(t_var,'var')
    error('%s/%s belum ada. Jalankan %s terlebih dahulu.', ...
          script_var, t_var, run_hint);
end

t_slx   = sig.Time;
slx_val = squeeze(sig.Data) * conv;     % rad->deg utk roll/pitch/yaw; alt tetap meter

script_t   = eval(t_var);
script_val = eval(script_var);

%% --- Plot ---
figure('Color','w','Position',[100 100 820 460]); hold on; grid on;
plot(t_slx,   slx_val,    'b-',  'LineWidth', 1.6);
plot(script_t, script_val, 'r--', 'LineWidth', 1.3);
yline(ref_val, 'k:', sprintf('Referensi %.1f %s', ref_val, unit));
legend('Model Simulink', 'Script MATLAB', 'Location', 'southeast');
xlabel('Waktu (s)'); ylabel(axis_label);
title(sprintf('Validasi-silang Step Response %s: Simulink vs Script', title_label));
set(gca, 'FontSize', 11);

%% --- Hitung selisih RMS (interpolasi ke grid script) ---
slx_i   = interp1(t_slx, slx_val, script_t, 'linear', 'extrap');
rms_err = sqrt(mean((slx_i - script_val).^2, 'omitnan'));
max_err = max(abs(slx_i - script_val));

% Ambang batas RELATIF thd referensi (generalisasi lintas satuan
% deg/meter): <2% = SANGAT BAIK, <5% = BAIK, selebihnya PERIKSA.
thresh_baik      = 0.02 * abs(ref_val);
thresh_cukup     = 0.05 * abs(ref_val);

fprintf('\n=== Hasil Validasi-Silang (%s) ===\n', upper(title_label));
fprintf('  RMS selisih  : %.4f %s\n', rms_err, unit);
fprintf('  Selisih maks : %.4f %s\n', max_err, unit);
if rms_err < thresh_baik
    fprintf('  --> KESESUAIAN SANGAT BAIK (RMS < %.2f%% referensi).\n', 2);
    fprintf('      Implementasi Simulink konsisten dgn analisis script.\n');
elseif rms_err < thresh_cukup
    fprintf('  --> Kesesuaian BAIK (perbedaan kecil: numerik/solver).\n');
else
    fprintf('  --> PERIKSA: parameter atau skenario Simulink mungkin\n');
    fprintf('      belum sama dengan script (cek RC_INPUT sesuai tabel\n');
    fprintf('      di komentar awal skrip ini utk sumbu "%s").\n', AXIS);
end