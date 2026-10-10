clear; clc; monocopter_params;

t_total = 10;
N = round(t_total/dt);
t_sy = (0:N-1)*dt;

yaw_ref = deg2rad(30);
alt_ref = alt_ref_const;

x = zeros(8,1);
x(7) = alt_ref;
x(8) = 0;

int_y = 0;

int_a = Kp_alt * alt_ref / Ki_alt;

MAX_RATE = deg2rad(250);

yaw_sy      = zeros(1,N);
alt_sy      = zeros(1,N);
throttle_sy = zeros(1,N);
w_omega_sy  = zeros(1,N);
d_yaw_sy    = zeros(1,N);

for k = 1:N
    psi  = x(5);
    dpsi = clamp(x(6), -MAX_RATE, MAX_RATE);
    z    = max(0, x(7));
    dz   = x(8);

    eA = alt_ref - z;
    int_a = clamp(int_a + eA*dt, -pid_alt_int_max, pid_alt_int_max);
    uA = clamp(-Kp_alt*z - Kd_alt*dz + Ki_alt*int_a, ...
               -pid_alt_force_max, pid_alt_force_max);

    throttle = clamp(throttle_hover + uA*F_to_throttle, 0.05, 1);

    eY = yaw_ref - psi;
    int_y = clamp(int_y + eY*dt, -pid_yaw_int_max, pid_yaw_int_max);
    uY = clamp(-Kp_yaw*psi - Kd_yaw*dpsi + Ki_yaw*int_y, ...
               -pid_yaw_torque_max, pid_yaw_torque_max);

    omega_for_w = throttle * omega_max;
    w_omega = max((omega_for_w/omega_hover)^2, 0.05);
    uY_c = uY / w_omega;

    delta_yaw = uY_c/(4*Ky_hover) + yaw_ff;
    delta_yaw = clamp(delta_yaw, -pid_out_max_y, pid_out_max_y);
    d_yaw_sy(k) = rad2deg(delta_yaw);

    omega = throttle * omega_max;
    FT = kT * omega^2;
    Q_current = c_q * FT;
    tau_yaw = 4*Ky_hover*w_omega*delta_yaw - Q_current - b_aero*dpsi;
    ddpsi = tau_yaw / Izz;
    ddz   = -g + FT/m;
    ddpsi = clamp(ddpsi, -150, 150);
    ddz   = clamp(ddz,  -15, 15);

    x(5) = psi + dt*dpsi;
    x(6) = clamp(dpsi + dt*ddpsi, -MAX_RATE, MAX_RATE);
    x(7) = max(0, x(7) + dt*dz);
    x(8) = dz + dt*ddz;
    if x(7) == 0, x(8) = max(0, x(8)); end

    yaw_sy(k)      = rad2deg(x(5));
    alt_sy(k)      = x(7);
    throttle_sy(k) = throttle;
    w_omega_sy(k)  = w_omega;
end

ss   = yaw_sy(end);
peak = max(yaw_sy);
OS   = ((peak - 30)/30) * 100;
if OS < 0; OS = 0; end

band = 0.02 * 30;
idx  = find(abs(yaw_sy - 30) > band);
ts   = (isempty(idx)) * 0 + (~isempty(idx)) * (idx(end) * dt);

fprintf('=== Step Response Yaw Tracking dari Hover Stabil, Umpan-Balik-Keadaan ===\n');
fprintf('  Steady-state      : %.3f deg (target 30)\n', ss);
fprintf('  Overshoot         : %.2f %%  (prediksi teoretis zeta=0,85: ~0,61%%)\n', OS);
fprintf('  Settling Time 2%%  : %.3f s\n', ts);
fprintf('  Throttle min-maks : %.4f -- %.4f (hover=%.4f)\n', ...
    min(throttle_sy), max(throttle_sy), throttle_hover);
fprintf('  w(omega) min-maks : %.4f -- %.4f (nominal desain=1)\n', ...
    min(w_omega_sy), max(w_omega_sy));
fprintf('  Defleksi Yaw maks : %.3f deg (batas=%.1f deg)\n', ...
    max(abs(d_yaw_sy)), rad2deg(pid_out_max_y));

figure('Color','w');
hold on; grid on;
plot(t_sy, yaw_sy, 'g-', 'LineWidth', 1.8);
yline(30, 'k--');
xlabel('Waktu (s)');
ylabel('Yaw \psi (\circ)');
title('Step Response Yaw Tracking 30° dari Hover Stabil — Bentuk Umpan-Balik-Keadaan (BAB III)');
legend('\psi (script, diperbaiki)', 'Referensi 30°', 'Location','best');

function y = clamp(x, lo, hi); y = max(lo, min(hi, x)); end