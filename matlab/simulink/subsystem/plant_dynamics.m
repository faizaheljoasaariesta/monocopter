function dxdt = plant_dynamics(servo1, servo2, servo3, servo4, throttle, x_state, m, g, Ixx, Iyy, Izz, kT, Ks_hover, omega_max, b_aero, c_q, kappa_y, omega_hover)
  % --- Unpack state --- 
  phi   = x_state(1);  dphi   = x_state(2);
  theta = x_state(3);  dtheta = x_state(4);
  psi   = x_state(5);  dpsi   = x_state(6);
  z     = x_state(7);  dz     = x_state(8);
  
  % --- Gaya angkat motor --- 
  omega = throttle * omega_max;
  FT    = kT * omega^2;
  
  % --- Torsi reaksi propeller (Hukum III Newton) --- 
  Q_reaction = c_q * FT;
  
  % --- Efektivitas sirip terhadap prop wash: w(omega) --- 
  w_omega = (omega/omega_hover)^2;
  
  % --- Torsi sirip: MODEL LINEAR Ks_hover*w(omega)*kappa_y + torsi
  tau_roll  = Ks_hover*w_omega*(servo1 - servo3) - b_aero*dphi;
  tau_pitch = Ks_hover*w_omega*(servo2 - servo4) - b_aero*dtheta;
  tau_yaw   = Ks_hover*w_omega*kappa_y*(servo1+servo2+servo3+servo4) - Q_reaction - b_aero*dpsi;
  
  % --- Persamaan Euler (rotasi benda tegar) --- 
  ddphi   = ((Iyy-Izz)*dtheta*dpsi + tau_roll ) / Ixx;
  ddtheta = ((Izz-Ixx)*dphi  *dpsi + tau_pitch) / Iyy;
  ddpsi   = ((Ixx-Iyy)*dphi*dtheta + tau_yaw  ) / Izz;
  
  % --- Translasi vertikal (Newton) --- 
  ddz = -g + cos(phi)*cos(theta)*FT/m;
  
  % --- Saturasi fisik (mencegah divergensi numerik) --- 
  ddphi   = max(-150, min(150, ddphi));
  ddtheta = max(-150, min(150, ddtheta));
  ddpsi   = max(-80,  min(80,  ddpsi));
  ddz     = max(-15,  min(15,  ddz));
  
  % --- Susun turunan state --- 
  dxdt = [dphi; ddphi; dtheta; ddtheta; dpsi; ddpsi; dz; ddz];
  
  % Ground constraint (tidak menembus tanah)
  if z <= 0 && dz < 0
      dxdt(7) = 0;
      dxdt(8) = max(0, dxdt(8));
  end
  end
  