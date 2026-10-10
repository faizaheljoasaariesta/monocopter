function [phi_est, theta_est] = kalman_update(tau_roll, tau_pitch, accel_roll, accel_pitch, b_roll, b_pitch)
  
  % PENJELASAN VARIABEL:
  
  %   tau_roll    = torsi kendali roll siklus sebelumnya [N.m] (dari PID_CONTROLLER)
  
  %   tau_pitch   = torsi kendali pitch siklus sebelumnya [N.m]
  
  %   accel_roll  = sudut roll dari accelerometer [rad] (noisy, bisa drift)
  
  %   accel_pitch = sudut pitch dari accelerometer [rad] (noisy, bisa drift)
  
  %   b_roll      = 1/Ixx [rad/s^2 per N.m] (Parameter Argument, dari monocopter_params.m)
  
  %   b_pitch     = 1/Iyy [rad/s^2 per N.m] (= b_roll karena Ixx=Iyy, tetap
  %                 diberikan terpisah untuk keumuman/keterbacaan)
  
  %   phi_est     = estimasi sudut roll [rad] (hasil filter, bersih)
  
  %   theta_est   = estimasi sudut pitch [rad] (hasil filter, bersih)
  
  
  % persistent: variabel yang bertahan nilainya antar pemanggilan fungsi
  
  persistent x_roll P_roll x_pitch P_pitch initialized
  
  
  % --- Inisialisasi saat pertama kali dipanggil --------------
  
  if isempty(initialized)
  
      x_roll  = [0; 0];  % State roll awal: [sudut=0; rate=0]
  
      P_roll  = eye(2);  % Error covariance awal = identitas
  
      x_pitch = [0; 0];
  
      P_pitch = eye(2);
  
      initialized = true;
  
  end
  
  
  % --- Parameter Kalman --------
  
  dt  = 0.005;                       % Sample time [s]
  
  Q   = diag([0.001, 0.003]);        % Process noise: [angle_noise; rate_noise]
  
  R   = 0.03;                        % Measurement noise (accelerometer)
  
  
  % --- Matriks State-Space --
  
  % State: [sudut; angular_rate]
  
  % Model: sudut_baru = sudut_lama + dt*rate ; rate_baru = rate_lama + dt*b*tau\
  
  A = [1, dt; 0, 1];               % State transition matrix
  
  C = [1, 0];                      % Observation matrix (hanya sudut yang diukur accel, TIDAK berubah)
  
  B_roll  = [dt^2/2*b_roll;  dt*b_roll];
  
  B_pitch = [dt^2/2*b_pitch; dt*b_pitch];
  
  
  % ============================================================
  % ROLL KALMAN FILTER
  % ============================================================
  
  
  % --- STEP 1: PREDICT --------------------------------------
  
  x_roll_pred = A * x_roll + B_roll * tau_roll;   % Prediksi state
  
  P_roll_pred = A * P_roll * A' + Q;              % Prediksi error covariance
  
  
  % --- STEP 2: UPDATE ---------------------------------------
  
  S_r = C * P_roll_pred * C' + R;                 % Innovation covariance
  
  K_r = P_roll_pred * C' / S_r;                   % Kalman Gain
  
  innovation_r = accel_roll - C * x_roll_pred;    % Residual (perbedaan pred vs ukur)
  
  x_roll  = x_roll_pred + K_r * innovation_r;     % Update state
  
  P_roll  = (eye(2) - K_r * C) * P_roll_pred;     % Update error covariance
  
  
  % --- Output roll -----------------------------------------
  
  phi_est = x_roll(1);  % Ambil elemen pertama = estimasi sudut
  
  % ============================================================
  % PITCH KALMAN FILTER
  % ============================================================
  
  % --- STEP 1: PREDICT --------------------------------------
  
  x_pitch_pred = A * x_pitch + B_pitch * tau_pitch;
  
  P_pitch_pred = A * P_pitch * A' + Q;
  
  % --- STEP 2: UPDATE ---------------------------------------
  
  S_p = C * P_pitch_pred * C' + R;
  
  K_p = P_pitch_pred * C' / S_p;
  
  innovation_p = accel_pitch - C * x_pitch_pred;
  
  x_pitch  = x_pitch_pred + K_p * innovation_p;
  
  P_pitch  = (eye(2) - K_p * C) * P_pitch_pred;
  
  % --- Output pitch -----------------------------------------
  
  theta_est = x_pitch(1);
  
  end
  