clear; clc; monocopter_params;

t_total = 10;
N = round(t_total/dt);
t_sa = (0:N-1)*dt;

alt_ref = alt_ref_const;

x = zeros(8,1);
int_a = 0;

MAX_RATE = deg2rad(250);

alt_sa = zeros(1,N);
thr_sa = zeros(1,N);

for k = 1:N
    z  = max(0, x(7));
    dz = x(8);

    eA = alt_ref - z;
    int_a = clamp(int_a + eA*dt, -pid_alt_int_max, pid_alt_int_max);
    uA = clamp(-Kp_alt*z - Kd_alt*dz + Ki_alt*int_a, ...
               -pid_alt_force_max, pid_alt_force_max);

    throttle = clamp(throttle_hover + uA*F_to_throttle, 0.05, 1);

    s_common = clamp(yaw_ff, -servo_max, servo_max);
    s1=s_common; s2=s_common; s3=s_common; s4=s_common;

    omega = throttle * omega_max;
    FT = kT * omega^2;
    ddz = clamp(-g + FT/m, -15, 15);

    x(7) = max(0, x(7) + dt*dz);
    x(8) = dz + dt*ddz;
    if x(7) == 0, x(8) = max(0, x(8)); end

    alt_sa(k) = x(7);
    thr_sa(k) = throttle*100;
end

ss = alt_sa(end);
peak = max(alt_sa);
OS = (peak - alt_ref)/alt_ref * 100;
band = 0.02*alt_ref;
idx = find(abs(alt_sa - alt_ref) > band);
ts = (isempty(idx))*0 + (~isempty(idx))*(idx(end)*dt);

fprintf('=== Step Response Altitude dari z=0, bentuk umpan-balik-keadaan ===\n');
fprintf('  Steady-state : %.3f m (target %.1f)\n', ss, alt_ref);
fprintf('  Overshoot    : %.2f %%  (prediksi teoretis zeta=0,90: ~0,15%%)\n', OS);
fprintf('  Settling 2%%  : %.3f s\n', ts);

figure('Color','w'); hold on; grid on;
plot(t_sa, alt_sa, 'b-', 'LineWidth', 1.5);
yline(alt_ref, 'k--');
xlabel('Waktu (s)'); ylabel('Altitude z (m)');
title('Step Response Altitude 3 m — Bentuk Umpan-Balik-Keadaan (BAB III)');
legend('z (script, diperbaiki)', 'Referensi 3 m', 'Location','southeast');

function y = clamp(x, lo, hi); y = max(lo, min(hi, x)); end