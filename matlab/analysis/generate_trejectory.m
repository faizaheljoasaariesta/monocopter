clear; clc; monocopter_params; close all;

clearvars C s L G Gm Pm pcl ii nm bb Kp_ Ki_ Kd_ axes_ver info Acl Bcl Ccl sys_cl;

%% --- 1. MUAT HASIL SIMULASI -----------------------------------
hasilFile = 'hasil_simulasi_full.mat';
if ~exist(hasilFile, 'file')
  error(['Berkas "%s" tidak ditemukan.\nJalankan simulasi_full_system.m ' ...
         'terlebih dahulu.'], hasilFile);
end
S = load(hasilFile);
if ~isfield(S, 'sim_results')
  error('Struktur "sim_results" tidak ditemukan di dalam %s.', hasilFile);
end
R = S.sim_results;
t_vec = R.time(:)';
fprintf('Berhasil memuat %d sampel dari %s.\n', numel(t_vec), hasilFile);

%% --- 2. GAYA VISUAL GLOBAL ------------------------------------
set(groot, 'DefaultAxesFontName',  'Times New Roman');
set(groot, 'DefaultTextFontName',  'Times New Roman');
set(groot, 'DefaultAxesFontSize',  11);
set(groot, 'DefaultTextFontSize',  11);
set(groot, 'DefaultAxesLineWidth', 1.0);
set(groot, 'DefaultLineLineWidth', 2.0);
set(groot, 'DefaultAxesBox',       'on');
set(groot, 'DefaultFigureColor',   'w');

Pal.blue   = [0.0000, 0.4470, 0.7410];
Pal.red    = [0.8500, 0.3250, 0.0980];
Pal.green  = [0.4660, 0.6740, 0.1880];
Pal.orange = [0.9290, 0.6940, 0.1250];
Pal.purple = [0.4940, 0.1840, 0.5560];
Pal.gray   = [0.60,   0.60,   0.60];

outDir = 'figures_output';
if ~exist(outDir, 'dir'); mkdir(outDir); end

%% --- 3. DETEKSI OTOMATIS WAKTU PERGANTIAN FASE ----------------
refs = [R.ref_roll(:), R.ref_pitch(:), R.ref_yaw(:), R.ref_alt(:)];
change_idx = find(any(abs(diff(refs, 1, 1)) > 1e-9, 2)) + 1;
t_fases    = unique(round(t_vec(change_idx), 6));

%% ============================================================
%% GAMBAR 4.8 - PELACAKAN LINTASAN TERINTEGRASI
%% ============================================================
fig1 = figure('Name', 'Gambar 4.8 - Respons Pelacakan Lintasan', ...
            'Position', [100, 50, 1100, 850], 'Color', 'w');

ref48  = {R.ref_roll, R.ref_pitch, R.ref_yaw, R.ref_alt};
act48  = {R.phi_deg,  R.theta_deg, R.psi_deg, R.altitude_m};
est48  = {R.phi_est,  R.theta_est, [],        []};
estCol = {Pal.green,  Pal.purple,  [],        []};
ylab48 = {'Roll, \phi (^\circ)', 'Pitch, \theta (^\circ)', ...
        'Yaw, \psi (^\circ)',  'Altitude, z (m)'};
ylim48 = {[-5 15], [-5 15], [-10 40], [-1 4]};

for k = 1:4
  subplot(4,1,k); hold on;
  plot(t_vec, ref48{k}, '--', 'Color', Pal.red,  'LineWidth', 1.5);
  plot(t_vec, act48{k}, '-',  'Color', Pal.blue, 'LineWidth', 1.8);
  if ~isempty(est48{k})
      plot(t_vec, est48{k}, ':', 'Color', estCol{k}, 'LineWidth', 1.5);
      legend('Referensi', 'Aktual', 'Estimasi Kalman', ...
             'Location', 'northeast', 'FontSize', 8, 'Box', 'on');
  else
      legend('Referensi', 'Aktual', ...
             'Location', 'northeast', 'FontSize', 8, 'Box', 'on');
  end
  add_phase_lines(t_fases, Pal.gray);
  grid on; ax = gca; ax.GridAlpha = 0.25;
  ax.XMinorGrid = 'on'; ax.YMinorGrid = 'on'; ax.MinorGridAlpha = 0.15;
  ax.FontSize = 9;
  ylabel(ylab48{k}); xlim([0 28]); ylim(ylim48{k});
  if k == 1
      title('Gambar 4.8. Respons Pelacakan Lintasan Terintegrasi Empat Sumbu Kendali', ...
            'FontSize', 13, 'FontWeight', 'bold');
  end
  if k == 4; xlabel('Waktu (detik)'); end
end
set(fig1, 'Renderer', 'painters');

%% ============================================================
%% GAMBAR 4.9 - ZOOM-IN FASE KRITIS
%% ============================================================
fig2 = figure('Name', 'Gambar 4.9 - Zoom-In Fase Kritis', ...
            'Position', [60, 120, 1550, 500], 'Color', 'w');

cfg(1) = struct('ttl','Manuver Roll Step',     'yl','Roll, \phi (^\circ)',    ...
  'win',[6.8 10.5],  'ref',R.ref_roll,  'act',R.phi_deg,    'est',R.phi_est,   'ecol',Pal.green,  'tgt',12, 'ylim',[-2 14]);
cfg(2) = struct('ttl','Manuver Pitch Step',    'yl','Pitch, \theta (^\circ)', ...
  'win',[11.8 15.5], 'ref',R.ref_pitch, 'act',R.theta_deg,  'est',R.theta_est, 'ecol',Pal.purple, 'tgt',10, 'ylim',[-2 12]);
cfg(3) = struct('ttl','Manuver Yaw Heading',   'yl','Yaw, \psi (^\circ)',     ...
  'win',[16.8 20.5], 'ref',R.ref_yaw,   'act',R.psi_deg,    'est',[],          'ecol',[],         'tgt',30, 'ylim',[-2 35]);
cfg(4) = struct('ttl','Manuver Altitude Step', 'yl','Altitude, z (m)',        ...
  'win',[1.5 6.5],   'ref',R.ref_alt,   'act',R.altitude_m, 'est',[],          'ecol',[],         'tgt',3,  'ylim',[-0.2 3.5]);

metrik = struct('Mp', cell(1,4), 'Ts', cell(1,4), 'dur', cell(1,4));
for k = 1:4
  c = cfg(k);
  axk = subplot(1,4,k); hold on;
  idx = (t_vec >= c.win(1)) & (t_vec <= c.win(2));
  plot(t_vec(idx), c.ref(idx), '--', 'Color', Pal.red,  'LineWidth', 1.5);
  plot(t_vec(idx), c.act(idx), '-',  'Color', Pal.blue, 'LineWidth', 2.5);
  if ~isempty(c.est)
      plot(t_vec(idx), c.est(idx), ':', 'Color', c.ecol, 'LineWidth', 2.0);
      legend('Referensi', 'Respons Aktual', 'Estimasi Kalman', ...
             'Location', 'southeast', 'FontSize', 8, 'Box', 'on');
  else
      legend('Referensi', 'Respons Aktual', ...
             'Location', 'southeast', 'FontSize', 8, 'Box', 'on');
  end
  grid on; axk.GridAlpha = 0.30; axk.FontSize = 9;
  axk.XMinorGrid = 'on'; axk.YMinorGrid = 'on'; axk.MinorGridAlpha = 0.15;
  xlabel('Waktu (s)'); ylabel(c.yl); ylim(c.ylim);
  title(c.ttl, 'FontSize', 11, 'FontWeight', 'bold');

  t_step = find_step_start(t_vec, c.ref, c.tgt);
  [Mp, Ts_, t_e] = step_metrics_from_reference(t_vec, c.act, c.ref, t_step);
  annotate_metrics(Mp, Ts_, t_e - t_step);
  metrik(k).Mp = Mp; metrik(k).Ts = Ts_; metrik(k).dur = t_e - t_step;
  hold off;
end
sgtitle('Gambar 4.9. Perbesaran (Zoom-In) Respons Transien pada Fase Manuver Kritis', ...
      'FontSize', 13, 'FontWeight', 'bold');
set(fig2, 'Renderer', 'painters');

%% ============================================================
%% GAMBAR 4.10 - DIAGRAM BODE OPEN-LOOP (BERFILTER DERIVATIF)
%% L(s) = b*(Kp + Ki/s + Kd*N*s/(s+N)) / s^2   (sama dgn params)
%% ============================================================
fig3 = figure('Name', 'Gambar 4.10 - Diagram Bode Open-Loop', ...
            'Position', [200, 200, 900, 750]);

w_bode = logspace(-1, 3, 500);
s = tf('s');

b_list     = [b_roll,  b_yaw,  b_alt];
kp_list    = [Kp_roll, Kp_yaw, Kp_alt];
ki_list    = [Ki_roll, Ki_yaw, Ki_alt];
kd_list    = [Kd_roll, Kd_yaw, Kd_alt];
axisNames  = {'Kanal Roll/Pitch (Sikap)', 'Kanal Yaw (Heading)', 'Kanal Altitude (Ketinggian)'};
axisColors = {Pal.blue, Pal.orange, Pal.green};

subplot(2,1,1); hold on; grid on; ax = gca; ax.GridAlpha = 0.3;
set(gca, 'XScale', 'log');
ylabel('Magnitudo, |L(j\omega)| (dB)');
title('Gambar 4.10. Diagram Bode Respons Frekuensi Open-Loop per Kanal Kendali', ...
    'FontSize', 12, 'FontWeight', 'bold');
yline(0, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');

subplot(2,1,2); hold on; grid on; ax = gca; ax.GridAlpha = 0.3;
set(gca, 'XScale', 'log');
ylabel('Fase, \angle L(j\omega) (\circ)'); xlabel('Frekuensi (rad/s)');
yline(-180, '--', 'Color', Pal.red, 'LineWidth', 1.2, 'HandleVisibility', 'off');

marginText = strings(1, numel(axisNames));
fprintf('\n============================================================\n');
fprintf('  MARGIN KESTABILAN OPEN-LOOP (filter derivatif N = %g)\n', N_filter);
fprintf('============================================================\n');

for i = 1:numel(axisNames)
  b = b_list(i); kp = kp_list(i); ki = ki_list(i); kd = kd_list(i);

  L = b * (kp + ki/s + kd*N_filter*s/(s + N_filter)) / s^2;

  [mag, phase] = bode(L, w_bode);
  mag_db    = 20*log10(squeeze(mag));
  phase_deg = squeeze(phase);

  [Gm, Pm, Wcg, Wcp] = margin(L);
  td = deg2rad(Pm) / Wcp;

  subplot(2,1,1);
  plot(w_bode, mag_db, '-', 'Color', axisColors{i}, 'LineWidth', 2, ...
       'DisplayName', axisNames{i});
  plot(Wcp, 0, 'o', 'Color', axisColors{i}, 'MarkerFaceColor', axisColors{i}, ...
       'MarkerSize', 5, 'HandleVisibility', 'off');

  subplot(2,1,2);
  plot(w_bode, phase_deg, '-', 'Color', axisColors{i}, 'LineWidth', 2, ...
       'DisplayName', axisNames{i});

  if isinf(Gm)
      gmStr = 'GM=\infty';  gmCmd = Inf;
  else
      gmCmd = 20*log10(Gm);
      gmStr = sprintf('GM=%.1f dB', gmCmd);
  end
  marginText(i) = sprintf('%s: PM=%.1f^{\\circ}, %s', axisNames{i}, Pm, gmStr);

  fprintf('%-30s PM=%6.2f deg | w_gc=%7.3f rad/s | GM=%7.2f dB | w_pc=%6.3f rad/s | t_d=%.4f s\n', ...
          axisNames{i}, Pm, Wcp, gmCmd, Wcg, td);
end
fprintf('(GM negatif = conditionally stable; kestabilan dibuktikan dari pole di LHP)\n');

subplot(2,1,1);
legend('Location', 'southwest', 'FontSize', 8.5, 'Box', 'on');
xlim([w_bode(1), w_bode(end)]);
subplot(2,1,2);
legend('Location', 'southwest', 'FontSize', 8.5, 'Box', 'on');
xlim([w_bode(1), w_bode(end)]);

annotation('textbox', [0.10, 0.005, 0.85, 0.05], ...
  'String', strjoin(marginText, '   |   '), ...
  'FontSize', 8.5, 'EdgeColor', 'none', 'HorizontalAlignment', 'center');

%% ============================================================
%% EKSPOR OTOMATIS (PNG 600 dpi + PDF vektor)
%% ============================================================
figs     = [fig1, fig2, fig3];
figNames = {'Gambar_4_8_PelacakanLintasan', 'Gambar_4_9_ZoomInFaseKritis', ...
          'Gambar_4_10_DiagramBode'};
for i = 1:numel(figs)
  exportgraphics(figs(i), fullfile(outDir, [figNames{i} '.png']), 'Resolution', 600);
  exportgraphics(figs(i), fullfile(outDir, [figNames{i} '.pdf']), 'ContentType', 'vector');
  fprintf('Tersimpan: %s (.png & .pdf)\n', figNames{i});
end
fprintf('\nSemua gambar telah diekspor ke folder "%s/".\n', outDir);

%% ============================================================
%% STEADY-STATE ERROR AKTUAL (dari respons aktual, bukan estimasi DKF)
%% Rata-rata 20% terakhir tiap jendela fase.
%% Jendela altitude = 5,0-7,0 s (hover SETELAH settling, sebelum roll).
%% ============================================================
ss_fraction = 0.20;
ess_roll  = ess_window(t_vec, R.phi_deg,    R.ref_roll,  7.0, 10.0, ss_fraction);
ess_pitch = ess_window(t_vec, R.theta_deg,  R.ref_pitch, 12.0, 15.0, ss_fraction);
ess_yaw   = ess_window(t_vec, R.psi_deg,    R.ref_yaw,   17.0, 19.5, ss_fraction);
ess_alt   = ess_window(t_vec, R.altitude_m, R.ref_alt,   5.0,  7.0,  ss_fraction);

fprintf('\n============================================================\n');
fprintf('             STEADY-STATE ERROR AKTUAL\n');
fprintf('============================================================\n');
fprintf('Roll     : e_ss = %+0.6f deg\n', ess_roll);
fprintf('Pitch    : e_ss = %+0.6f deg\n', ess_pitch);
fprintf('Yaw      : e_ss = %+0.6f deg\n', ess_yaw);
fprintf('Altitude : e_ss = %+0.6f m\n',   ess_alt);
fprintf('============================================================\n');

fprintf('\n--- Ringkasan Mp / Ts (Tabel metrik Bab 4) ---\n');
nm4 = {'Roll','Pitch','Yaw','Altitude'};
for k = 1:4
  if isfinite(metrik(k).Ts)
      fprintf('%-9s Mp=%.2f %%  Ts=%.2f s\n', nm4{k}, metrik(k).Mp, metrik(k).Ts);
  else
      fprintf('%-9s Mp=%.2f %%  Ts > %.2f s (belum settling dalam fase)\n', ...
              nm4{k}, metrik(k).Mp, metrik(k).dur);
  end
end

%% ============================================================
%% FUNGSI LOKAL (HARUS DI AKHIR BERKAS)
%% ============================================================

function add_phase_lines(t_fases, color)
  for i = 1:numel(t_fases)
      xline(t_fases(i), ':', 'Color', color, 'HandleVisibility', 'off');
  end
end

function e = ess_window(t, y, ref, t0, t1, frac)
  idx = (t >= t0) & (t < t1);
  yy = y(idx);  rr = ref(idx);
  n  = numel(yy);
  last = max(1, round((1-frac)*n)):n;
  e = mean(rr(last)) - mean(yy(last));
end

function t_step = find_step_start(t, ref, target)
  t = t(:); ref = ref(:);
  if numel(t) ~= numel(ref)
      error('t dan ref harus memiliki panjang yang sama.');
  end
  dref = [0; abs(diff(ref))];
  tol  = max(1e-9, 1e-6 * max(1, abs(target)));
  k = find((abs(ref - target) <= tol) & (dref > tol), 1, 'first');
  if isempty(k)
      error('Tidak ditemukan perubahan referensi menuju %.6g.', target);
  end
  t_step = t(k);
end

function [Mp_pct, Ts, t_end] = step_metrics_from_reference(t, y, ref, t_step)
  t = t(:); y = y(:); ref = ref(:);
  valid = isfinite(t) & isfinite(y) & isfinite(ref);
  t = t(valid); y = y(valid); ref = ref(valid);

  k0 = find(t >= t_step, 1, 'first');
  if isempty(k0)
      error('t_step = %.3f s berada di luar rentang data.', t_step);
  end
  y0 = ref(k0);
  if k0 > 1; y0 = ref(k0-1); end
  yf = ref(k0);
  step_size = yf - y0;
  if abs(step_size) < 1e-12
      error('Tidak ada perubahan referensi valid pada t_step = %.3f s.', t_step);
  end

  ref_tol = max(1e-9, 1e-6 * abs(step_size));
  k_end = numel(t);
  for k = k0+1:numel(t)
      if abs(ref(k) - yf) > ref_tol
          k_end = k - 1; break;
      end
  end
  if k_end <= k0
      error('Jendela step pada t_step = %.3f s tidak valid.', t_step);
  end
  t_end = t(k_end);

  idx = (t >= t_step) & (t <= t_end);
  tt = t(idx); yy = y(idx);

  if step_size > 0
      Mp_pct = max(0, (max(yy) - yf) / abs(step_size) * 100);
  else
      Mp_pct = max(0, (yf - min(yy)) / abs(step_size) * 100);
  end

  within = abs(yy - yf) <= 0.02 * abs(step_size);
  outside_idx = find(~within);
  if isempty(outside_idx)
      Ts = 0;
  elseif outside_idx(end) == numel(tt)
      Ts = NaN;
  else
      Ts = tt(outside_idx(end) + 1) - t_step;
  end
end

function annotate_metrics(Mp, Ts, window_duration)
  if nargin < 3 || isempty(window_duration); window_duration = NaN; end
  if isfinite(Mp) && isfinite(Ts)
      lbl = sprintf('M_p \\approx %.1f%%  |  T_s \\approx %.2f s', Mp, Ts);
  elseif isfinite(Mp) && isfinite(window_duration)
      lbl = sprintf('M_p \\approx %.1f%%  |  T_s > %.2f s', Mp, window_duration);
  elseif isfinite(Mp)
      lbl = sprintf('M_p \\approx %.1f%%  |  T_s > fase', Mp);
  else
      error('M_p tidak dapat dihitung dari data simulasi.');
  end
  xl = xlim; yl = ylim;
  text(xl(1) + 0.04*diff(xl), yl(2) - 0.06*diff(yl), lbl, ...
       'FontSize', 8.5, 'FontWeight', 'bold', 'Color', 'k', ...
       'BackgroundColor', 'w', 'Margin', 2, 'Interpreter', 'tex', ...
       'VerticalAlignment', 'top');
end