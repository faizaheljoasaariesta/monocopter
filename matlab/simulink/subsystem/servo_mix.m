function [s1_raw, s2_raw, s3_raw, s4_raw] = servo_mix(u_roll, u_pitch, u_yaw, yaw_ff, throttle, omega_max, omega_hover, Ks_hover, Ky_hover)

  % Layout fisik servo (dilihat dari atas):

  %

  %         DEPAN

  %          [S2]          ← Servo 2, posisi 0° (depan)

  %   [S1]   [M]   [S3]   ← Servo 1 kiri (270°), Motor tengah, Servo 3 kanan (90°)

  %          [S4]          ← Servo 4, posisi 180° (belakang)

  %         BELAKANG

  %

  % Logika mixing:

  %   ROLL  kanan (+): S1 defleksi -, S3 defleksi +  → wahana miring kanan

  %   PITCH maju  (+): S2 defleksi +, S4 defleksi -  → wahana miring depan

  %   YAW   searah (+): semua servo defleksi searah   → wahana berputar

  %   YAW_FF: bias kolektif KONSTAN (titik hover) menawar torsi reaksi

  %   propeller -- sirip TIDAK PERNAH netral saat hover (biaya permanen

  %   konfigurasi single-rotor, lihat BAB II/IV)


  % --- Langkah 1: Gain scheduling -- efektivitas sirip vs throttle ---
  omega   = throttle * omega_max;
  w_omega = (omega/omega_hover)^2;
  w_omega = max(w_omega, 0.05);

  u_roll_c  = u_roll  / w_omega;
  u_pitch_c = u_pitch / w_omega;
  u_yaw_c   = u_yaw   / w_omega;

  % --- Langkah 2: Konversi torsi-ke-defleksi ---
  % delta = torsi / (efektivitas sirip), menghasilkan RADIAN sirip.
  delta_roll  = u_roll_c  / (2*Ks_hover);
  delta_pitch = u_pitch_c / (2*Ks_hover);
  delta_yaw   = u_yaw_c   / (4*Ky_hover);

  % --- Langkah 3: Mixing matrix + feedforward kolektif ---

  % Format: s = M * [delta_roll; delta_pitch; delta_yaw] + yaw_ff
  % (yaw_ff sudah dalam satuan radian, TANPA kompensasi tambahan)

  s1_raw =  delta_roll  +  0           +  delta_yaw  +  yaw_ff;   % Servo 1 (kiri, 270°)

  s2_raw =  0           +  delta_pitch +  delta_yaw  +  yaw_ff;   % Servo 2 (depan, 0°)

  s3_raw = -delta_roll  +  0           +  delta_yaw  +  yaw_ff;   % Servo 3 (kanan, 90°)

  s4_raw =  0           + -delta_pitch +  delta_yaw  +  yaw_ff;   % Servo 4 (belakang, 180°)


  % CATATAN: Saturation (clamp ±servo_max) dilakukan di Saturation block yang terpisah, bukan di sini. Ini memudahkan monitoring nilai sebelum saturasi.
end
