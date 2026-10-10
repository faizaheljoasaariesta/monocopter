clear; clc; monocopter_params;

t_total = 10;
N = round(t_total/dt);
t_sr = (0:N-1)*dt;

%% --- Langkah 1: Referensi KONSTAN (Constant Block Simulink) -
roll_ref = joy_roll_const * servo_max;
alt_ref  = alt_ref_const;

%% --- Langkah 2: Kondisi awal HOVER STABIL -------------------
x = zeros(8,1);
x(7) = alt_ref;
x(8) = 0;

int_r = 0;

int_a = 0; pe_a = 0;
MAX_ANG  = deg2rad(55);
MAX_RATE = deg2rad(250);

roll_sr      = zeros(1,N);
alt_sr       = zeros(1,N);
throttle_sr  = zeros(1,N);
w_omega_sr   = zeros(1,N);

for k = 1:N
    phi  = clamp(x(1), -MAX_ANG, MAX_ANG);
    dphi = clamp(x(2), -MAX_RATE, MAX_RATE);
    z    = max(0, x(7));
    dz   = x(8);

    %% --- Langkah 3: PID ROLL -- BENTUK UMPAN BALIK KEADAAN --
    eR = roll_ref - phi;
    int_r = clamp(int_r + eR*dt, -pid_roll_int_max, pid_roll_int_max);
    uR = clamp(-Kp_roll*phi - Kd_roll*dphi + Ki_roll*int_r, ...
               -pid_roll_torque_max, pid_roll_torque_max);

    %% --- Langkah 4: PID ALTITUDE (domain gaya, Newton) ------
    eA = alt_ref - z;
    int_a = clamp(int_a + eA*dt, -pid_alt_int_max, pid_alt_int_max);
    uA = clamp(Kp_alt*eA + Ki_alt*int_a + Kd_alt*(eA-pe_a)/dt, ...
               -pid_alt_force_max, pid_alt_force_max);
    pe_a = eA;

    %% --- Langkah 5: THROTTLE --------------------------------
    throttle = clamp(throttle_hover + uA*F_to_throttle, 0.05, 1);

    %% --- Langkah 6: GAIN SCHEDULING w(omega) ----------------
    omega_for_w = throttle * omega_max;
    w_omega = max((omega_for_w/omega_hover)^2, 0.05);
    uR_c = uR / w_omega;

    %% --- Langkah 7: SERVO MIXER (roll murni + feedforward kolektif) 
    delta_r = uR_c / (2*Ks_hover);
    s1 = clamp( delta_r + yaw_ff, -servo_max, servo_max);
    s3 = clamp(-delta_r + yaw_ff, -servo_max, servo_max);

    %% --- Langkah 8: PLANT PHYSICS ---------------------------
    omega = throttle * omega_max;
    FT = kT * omega^2;
    tau_roll = Ks_hover*w_omega*(s1 - s3) - b_aero*dphi;
    ddph = tau_roll / Ixx;
    ddz  = -g + cos(phi)*FT/m;
    ddph = clamp(ddph, -150, 150);
    ddz  = clamp(ddz,  -15, 15);

    %% --- Langkah 9: INTEGRASI EULER -------------------------
    x(1) = clamp(x(1) + dt*dphi, -MAX_ANG, MAX_ANG);
    x(2) = clamp(x(2) + dt*ddph, -MAX_RATE, MAX_RATE);
    x(7) = max(0, x(7) + dt*dz);
    x(8) = dz + dt*ddz;
    if x(7) == 0, x(8) = max(0, x(8)); end

    roll_sr(k)     = x(1)*180/pi;
    alt_sr(k)      = x(7);
    throttle_sr(k) = throttle;
    w_omega_sr(k)  = w_omega;
end

%% --- Analisis step response ---------------------------------
ss   = roll_sr(end);
peak = max(roll_sr);
OS   = (peak - 12)/12 * 100;
band = 0.02*12;
idx  = find(abs(roll_sr - 12) > band);
ts   = (isempty(idx))*0 + (~isempty(idx))*(idx(end)*dt);

fprintf('=== Step Response Roll dari HOVER STABIL, bentuk umpan-balik-keadaan ===\n');
fprintf('  Steady-state      : %.3f deg (target 12)\n', ss);
fprintf('  Overshoot         : %.2f %%  (prediksi teoretis zeta=0,85: ~0,6%%)\n', OS);
fprintf('  Settling 2%%       : %.3f s\n', ts);
fprintf('  Throttle min-maks : %.4f -- %.4f (hover=%.4f)\n', ...
        min(throttle_sr), max(throttle_sr), throttle_hover);
fprintf('  w(omega) min-maks : %.4f -- %.4f (nominal desain=1)\n', ...
        min(w_omega_sr), max(w_omega_sr));

%% --- Grafik -------------------------------------------------
figure('Color','w');
hold on; grid on;
plot(t_sr, roll_sr, 'g-', 'LineWidth', 1.8);
yline(12, 'k--');
xlabel('Waktu (s)'); ylabel('Roll \phi (\circ)');
title('Step Response Roll 12° dari Hover Stabil — Bentuk Umpan-Balik-Keadaan (BAB III)');
legend('\phi (script, diperbaiki)', 'Referensi 12°', 'Location','best');

function y = clamp(x, lo, hi); y = max(lo, min(hi, x)); end