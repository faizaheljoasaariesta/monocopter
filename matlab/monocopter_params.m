clear; clc;

%% --- 1. PARAMETER FISIK WAHANA (model kontrol) --------------
m      = 0.5;                                                   % Massa total [kg]
g      = 9.81;                                                  % Gravitasi [m/s^2]
R_body = 0.075;                                                 % radius badan [m]
L_body = 0.32;                                                  % tinggi badan [m]
Ixx    = (1/12)*m*(3*R_body^2 + L_body^2);                      % = 4,9698e-3
Iyy    = Ixx;
Izz    = 0.5*m*R_body^2;                                        % = 1,40625e-3

%% --- 2. MOTOR & PROPELLER (komponen nyata) ------------------
KV       = 2200;                                                % KV motor A2212/6T
V_batt   = 11.1;                                                % Li-ion 3S [V]
omega_max = 0.8*KV*V_batt*2*pi/60;                              % ~2045,8 rad/s (80% no-load)
r_prop   = 0.0635;                                              % radius prop 5043 (5 inci) [m]
rho      = 1.225;                                               % kerapatan udara [kg/m^3]
b_aero   = 1.0e-3;                                              % redaman aerodinamis rotasi [N.m.s/rad]
thrust_max_N = 0.850*g;                                         % thrust maks ~850g (A2212+5043 @ 3S)
kT     = thrust_max_N/omega_max^2;                              % = 1,9923e-6

%% --- 3. BATAS AKTUATOR --------------------------------------
servo_max_deg = 30;
servo_max     = deg2rad(servo_max_deg);                         % 0,5236 rad
servo_min     = -servo_max;

%% --- 4. KONDISI HOVER ---------------------------------------
omega_hover   = sqrt((m*g)/kT);                                 % ~1569,1 rad/s
throttle_hover= omega_hover/omega_max;                          % ~0,76696
q_dyn_hover   = 0.5*rho*(omega_hover*r_prop)^2;                 % ~6081 Pa
TWR           = thrust_max_N/(m*g);                             % ~1,70

%% --- 5. EFEKTIVITAS TORSI SIRIP (OTORITAS KONTROL FISIK) ----
ddphi_max = 12;                                                 % rad/s^2 target otoritas
Ks_hover  = Ixx*ddphi_max/servo_max;                            % ~0,1139 N.m/rad
Keff      = Ks_hover;
Kplant    = Keff/Ixx;

%% --- 5b. TORSI REAKSI PROPELLER & KOMPENSASI FEEDFORWARD YAW -
kappa_y  = 0.5;                                                 % faktor mixing kolektif yaw [-]
Ky_hover = kappa_y*Ks_hover;                                    % ~0,05695 N.m/rad

c_q = 0.005;                                                    % koefisien torsi reaksi propeller [m]

Q_hover = c_q * m * g;                                          % torsi reaksi propeller saat hover [N.m] ~0,02453
yaw_ff  = Q_hover / (4*Ky_hover);                               % ~0,1077 rad (~6,17 deg)

%% --- 5c. EFEKTIVITAS SIRIP TERHADAP PROP WASH: w(omega) -----
% w(omega) = (omega/omega_hover)^2, w = 1 tepat di hover. Dipakai di servo_mix.m SEBELUM konversi torsi-ke-defleksi.

%% --- 6. PID ROLL/PITCH/YAW: POLE-PLACEMENT PENUH ------------
alpha = 5;                                                      % rasio kutub tambahan thd wn

b_roll  = 1/Ixx;                                                % ~201,22 rad/s^2 per N.m
b_pitch = 1/Iyy;                                                % = b_roll (Ixx=Iyy)
b_yaw   = 1/Izz;                                                % ~711,11 rad/s^2 per N.m

wn = 3.5; zeta = 0.85;                                          % spesifikasi sikap (roll/pitch/yaw)

Kd_roll = (alpha+2*zeta)*wn/b_roll;                             % ~0,11654
Kp_roll = (1+2*alpha*zeta)*wn^2/b_roll;                         % ~0,57836
Ki_roll = alpha*wn^3/b_roll;                                    % ~1,06540
Kp_pitch = Kp_roll; Ki_pitch = Ki_roll; Kd_pitch = Kd_roll;     % Ixx=Iyy -> identik

Kd_yaw = (alpha+2*zeta)*wn/b_yaw;                               % ~0,03298
Kp_yaw = (1+2*alpha*zeta)*wn^2/b_yaw;                           % ~0,16365
Ki_yaw = alpha*wn^3/b_yaw;                                      % ~0,30146

Kplant_yaw_v2 = 4*Keff*kappa_y/Izz;                             % ~162,0 (basis lama, sudah digantikan b_yaw)

N_filter = 100;
pid_out_max_rp = servo_max;                                     % catatan: batas SIRIP, BUKAN batas keluaran PID
pid_out_max_y  = deg2rad(20);                                   % idem, batas sudut sirip yaw

%% --- 7. LOOP ALTITUDE: POLE-PLACEMENT PENUH + KONVERSI THROTTLE
b_alt  = 1/m;                                                   % = 2,0
wn_alt = 1.5; zeta_alt = 0.9;                                   % spesifikasi altitude

Kd_alt = (alpha+2*zeta_alt)*wn_alt/b_alt;                       % = 5,100
Kp_alt = (1+2*alpha*zeta_alt)*wn_alt^2/b_alt;                   % = 11,250
Ki_alt = alpha*wn_alt^3/b_alt;                                  % = 8,4375

Kalt_v2 = kT*omega_max^2/m;                                     % ~16,68 (basis lama)

F_to_throttle = 1/(2*kT*omega_hover*omega_max);                 % ~1/12,79 per Newton

pid_out_max_a = 0.40;                                           % catatan: ini batas Delta THROTTLE (setelah konversi F_to_throttle), BUKAN batas Delta F_T mentah.

%% --- 7a. BATAS KELUARAN PID TORSI-DOMAIN --------------------
pid_roll_torque_max  = 2*Ks_hover*servo_max;                    % ~0,11927 N.m
pid_pitch_torque_max = pid_roll_torque_max;                     % Ixx=Iyy -> identik
pid_yaw_torque_max   = 4*Ky_hover*pid_out_max_y;                % ~0,07952 N.m
pid_alt_force_max    = pid_out_max_a/F_to_throttle;             % ~5,1163 N

%% --- 7b. BATAS SATURASI INTEGRATOR -- DIHITUNG ULANG (v4) ---
MARGIN_INT = 1.5;                                               % margin keamanan 50% di atas kebutuhan minimum

roll_ref_max_uji  = servo_max;                                  % 30 deg
yaw_ref_max_uji   = servo_max;                                  % 30 deg (msh dlm domain sudut absolut)
alt_ref_max_uji   = 5.0;                                        % meter

pid_roll_int_max  = (Kp_roll * roll_ref_max_uji / Ki_roll ) * MARGIN_INT;  % ~0,4264
pid_pitch_int_max = pid_roll_int_max;                                      % identik (Ixx=Iyy)
pid_yaw_int_max   = (Kp_yaw  * yaw_ref_max_uji  / Ki_yaw  ) * MARGIN_INT;  % ~0,4264
pid_alt_int_max   = (Kp_alt  * alt_ref_max_uji  / Ki_alt  ) * MARGIN_INT;  % ~10,00

%% --- 8. KALMAN FILTER ----------------------------------------
Ts_kf = 0.005;
Ad_kf = [1 Ts_kf; 0 1];
Bd_roll_kf  = [Ts_kf^2/2; Ts_kf]*b_roll;                        % ~[2,515e-3; 1,006]
Bd_pitch_kf = Bd_roll_kf;                                       % Ixx=Iyy
C_kf  = [1 0];

Q_kf = diag([0.001, 0.003]);
R_kf = 0.03;
P0_kf= eye(2);

%% --- 9. NOISE SENSOR (IMU MPU-6050 dalam GY-87) --------------
gyro_noise_pwr  = 1.0e-6;                                       % (a) uji Kalman - gyro - 1.0e-6
accel_noise_pwr = 1.0e-4;                                       % (a) uji Kalman - accel (±8°) - 1.0e-4
gyro_noise_sim  = 1.0e-7;                                       % (b) simulasi kontrol penuh - gyro (±0,26°/s) - 1.0e-7
accel_noise_sim = 4.0e-6;                                       % (b) simulasi kontrol penuh - accel (±1,6°) - 4.0e-6

seed_gx = 23341; seed_gy = 23342;
seed_ar = 23343; seed_ap = 23344;

%% --- 10. WAKTU ------------------------------------------------
dt    = 0.005;  Ts = dt;  t_end = 28.0;                         % 28 s, 13 fase

%% --- 10b. ALIAS UNTUK BLOK SIMULINK ---------------------------
pid_roll_outmax  =  servo_max;     pid_roll_outmin  = -servo_max;
pid_pitch_outmax =  servo_max;     pid_pitch_outmin = -servo_max;
pid_yaw_outmax   =  pid_out_max_y; pid_yaw_outmin   = -pid_out_max_y;
pid_alt_outmax   =  pid_out_max_a; pid_alt_outmin   = -pid_out_max_a;

yaw_ref_default  = 0.0;
alt_ref_default  = 0.0;

joy_roll_const   = 0.4;
joy_pitch_const  = 0.0;
yaw_ref_const    = yaw_ref_default;
alt_ref_const    = 3.0;
throttle_const   = throttle_hover;
mode_const       = 0;

noise_seed_gx = seed_gx;  noise_seed_gy = seed_gy;
noise_seed_ar = seed_ar;  noise_seed_ap = seed_ap;

fprintf('=== Parameter monochopter v4 (m=0,5kg, PID umpan-balik-keadaan) ===\n');
fprintf('  m              = %.3f kg\n', m);
fprintf('  Ixx=Iyy        = %.4e kg.m^2 | Izz = %.4e kg.m^2\n', Ixx, Izz);
fprintf('  kT             = %.4e\n', kT);
fprintf('  omega_max      = %.1f rad/s | omega_hover = %.1f rad/s\n', omega_max, omega_hover);
fprintf('  throttle_hover = %.5f | TWR = %.2f\n', throttle_hover, TWR);
fprintf('  Ks_hover       = %.4f N.m/rad | Ky_hover = %.4f N.m/rad\n', Ks_hover, Ky_hover);
fprintf('  Q_hover        = %.4f N.m | yaw_ff = %.4f rad (%.2f deg)\n', Q_hover, yaw_ff, rad2deg(yaw_ff));
fprintf('  --- Gain plant torsi-domain ---\n');
fprintf('  b_roll=b_pitch = %.2f | b_yaw = %.2f | b_alt = %.2f\n', b_roll, b_yaw, b_alt);
fprintf('  alpha (pole placement) = %.1f\n', alpha);
fprintf('  --- PID roll/pitch (N.m/rad, N.m.s/rad) ---\n');
fprintf('  Kp=%.5f  Ki=%.5f  Kd=%.5f\n', Kp_roll, Ki_roll, Kd_roll);
fprintf('  --- PID yaw ---\n');
fprintf('  Kp=%.5f  Ki=%.5f  Kd=%.5f\n', Kp_yaw, Ki_yaw, Kd_yaw);
fprintf('  --- PID altitude (N/m, N.s/m) ---\n');
fprintf('  Kp=%.4f  Ki=%.4f  Kd=%.4f\n', Kp_alt, Ki_alt, Kd_alt);
fprintf('  F_to_throttle  = %.6f per Newton (1/%.3f)\n', F_to_throttle, 1/F_to_throttle);
fprintf('  --- Batas keluaran PID torsi-domain (Saturation Simulink) ---\n');
fprintf('  pid_roll/pitch_torque_max = %.5f N.m\n', pid_roll_torque_max);
fprintf('  pid_yaw_torque_max        = %.5f N.m\n', pid_yaw_torque_max);
fprintf('  pid_alt_force_max         = %.4f N\n', pid_alt_force_max);
fprintf('  --- Batas integrator (bentuk umpan-balik-keadaan, BARU v4) ---\n');
fprintf('  pid_roll_int_max  = %.5f  (thd referensi maks %.1f deg)\n', pid_roll_int_max, rad2deg(roll_ref_max_uji));
fprintf('  pid_pitch_int_max = %.5f  (identik roll, Ixx=Iyy)\n', pid_pitch_int_max);
fprintf('  pid_yaw_int_max   = %.5f  (thd referensi maks %.1f deg)\n', pid_yaw_int_max, rad2deg(yaw_ref_max_uji));
fprintf('  pid_alt_int_max   = %.5f  (thd referensi maks %.1f m)\n', pid_alt_int_max, alt_ref_max_uji);

if exist('ss','file')==2
    fprintf('\n--- Verifikasi step response (bentuk umpan-balik-keadaan) ---\n');
    axes_ver = {'roll', b_roll, Kp_roll, Ki_roll, Kd_roll; ...
                'yaw',  b_yaw,  Kp_yaw,  Ki_yaw,  Kd_yaw; ...
                'alt',  b_alt,  Kp_alt,  Ki_alt,  Kd_alt};

    for ii = 1:size(axes_ver,1)
        nm = axes_ver{ii,1}; bb = axes_ver{ii,2};
        Kp_ = axes_ver{ii,3}; Ki_ = axes_ver{ii,4}; Kd_ = axes_ver{ii,5};

        Acl = [0 1 0; -bb*Kp_ -bb*Kd_ bb*Ki_; -1 0 0];
        Bcl = [0;0;1];
        Ccl = [1 0 0];
        sys_cl = ss(Acl, Bcl, Ccl, 0);
        info = stepinfo(sys_cl);
        fprintf('  %-5s : Overshoot=%.2f%%  SettlingTime=%.3fs  RiseTime=%.3fs\n', ...
                nm, info.Overshoot, info.SettlingTime, info.RiseTime);
    end
    fprintf('(Overshoot roll/yaw seharusnya ~0,61%% dan alt ~0,15%% sesuai spesifikasi zeta desain)\n');

    if exist('tf','file')==2
        s=tf('s');
        fprintf('\n--- Verifikasi margin kestabilan (PM/GM, independen bentuk PID) ---\n');
        for ii = 1:size(axes_ver,1)
            nm = axes_ver{ii,1}; bb = axes_ver{ii,2};
            Kp_ = axes_ver{ii,3}; Ki_ = axes_ver{ii,4}; Kd_ = axes_ver{ii,5};
            G = bb/s^2;
            C = Kp_+Ki_/s+Kd_*N_filter*s/(s+N_filter);
            L = C*G; [Gm,Pm]=margin(L);
            pcl = pole(feedback(L,1));
            fprintf('  %-5s : PM=%.1f deg | ', nm, Pm);
            if isinf(Gm), fprintf('GM=inf | '); else, fprintf('GM=%.1f dB | ',20*log10(Gm)); end
            fprintf('semua pole di LHP? %s\n', mat2str(all(real(pcl)<0)));
        end
    end
end
fprintf('==================================================\n');