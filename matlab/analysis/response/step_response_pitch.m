clear; clc; monocopter_params;

t_total = 10;
N = round(t_total/dt);
t_sp = (0:N-1)*dt;

pitch_ref = deg2rad(10);
alt_ref   = alt_ref_const;

x = zeros(8,1);
x(7) = alt_ref;
x(8) = 0;

int_p = 0;

int_a = 0; pe_a = 0;
MAX_ANG  = deg2rad(55);
MAX_RATE = deg2rad(250);

pitch_sp     = zeros(1,N);
alt_sp       = zeros(1,N);
throttle_sp  = zeros(1,N);
w_omega_sp   = zeros(1,N);

for k = 1:N
    theta  = clamp(x(3), -MAX_ANG, MAX_ANG);
    dtheta = clamp(x(4), -MAX_RATE, MAX_RATE);
    z      = max(0, x(7));
    dz     = x(8);

    eP = pitch_ref - theta;
    int_p = clamp(int_p + eP*dt, -pid_pitch_int_max, pid_pitch_int_max);
    uP = clamp(-Kp_pitch*theta - Kd_pitch*dtheta + Ki_pitch*int_p, ...
               -pid_pitch_torque_max, pid_pitch_torque_max);

    eA = alt_ref - z;
    int_a = clamp(int_a + eA*dt, -pid_alt_int_max, pid_alt_int_max);
    uA = clamp(Kp_alt*eA + Ki_alt*int_a + Kd_alt*(eA-pe_a)/dt, ...
               -pid_alt_force_max, pid_alt_force_max);
    pe_a = eA;

    throttle = clamp(throttle_hover + uA*F_to_throttle, 0.05, 1);

    omega_for_w = throttle * omega_max;
    w_omega = max((omega_for_w/omega_hover)^2, 0.05);
    uP_c = uP / w_omega;

    delta_p = uP_c / (2*Ks_hover);
    s2 = clamp( delta_p + yaw_ff, -servo_max, servo_max);
    s4 = clamp(-delta_p + yaw_ff, -servo_max, servo_max);

    omega = throttle * omega_max;
    FT = kT * omega^2;
    tau_pitch = Ks_hover*w_omega*(s2 - s4) - b_aero*dtheta;
    ddth = tau_pitch / Iyy;
    ddz  = -g + cos(theta)*FT/m;
    ddth = clamp(ddth, -150, 150);
    ddz  = clamp(ddz,  -15, 15);

    x(3) = clamp(x(3) + dt*dtheta, -MAX_ANG, MAX_ANG);
    x(4) = clamp(x(4) + dt*ddth, -MAX_RATE, MAX_RATE);
    x(7) = max(0, x(7) + dt*dz);
    x(8) = dz + dt*ddz;
    if x(7) == 0, x(8) = max(0, x(8)); end

    pitch_sp(k)    = x(3)*180/pi;
    alt_sp(k)      = x(7);
    throttle_sp(k) = throttle;
    w_omega_sp(k)  = w_omega;
end

ss   = pitch_sp(end);
peak = max(pitch_sp);
OS   = (peak - 10)/10 * 100;
band = 0.02*10;
idx  = find(abs(pitch_sp - 10) > band);
ts   = (isempty(idx))*0 + (~isempty(idx))*(idx(end)*dt);

fprintf('=== Step Response Pitch dari HOVER STABIL, bentuk umpan-balik-keadaan ===\n');
fprintf('  Steady-state      : %.3f deg (target 10)\n', ss);
fprintf('  Overshoot         : %.2f %%  (prediksi teoretis zeta=0,85: ~0,6%%)\n', OS);
fprintf('  Settling 2%%       : %.3f s\n', ts);
fprintf('  Throttle min-maks : %.4f -- %.4f (hover=%.4f)\n', ...
        min(throttle_sp), max(throttle_sp), throttle_hover);
fprintf('  w(omega) min-maks : %.4f -- %.4f (nominal desain=1)\n', ...
        min(w_omega_sp), max(w_omega_sp));

figure('Color','w');
hold on; grid on;
plot(t_sp, pitch_sp, 'm-', 'LineWidth', 1.8);
yline(10, 'k--');
xlabel('Waktu (s)'); ylabel('Pitch \theta (\circ)');
title('Step Response Pitch 10° dari Hover Stabil — Bentuk Umpan-Balik-Keadaan (BAB III)');
legend('\theta (script, diperbaiki)', 'Referensi 10°', 'Location','best');

function y = clamp(x, lo, hi); y = max(lo, min(hi, x)); end