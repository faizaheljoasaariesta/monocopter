function [accel_roll, accel_pitch] = accel_sensor(phi, theta)

  % Mensimulasikan pembacaan sudut dari accelerometer
  
  % Di dunia nyata: sudut dihitung dari atan2(ay, sqrt(ax²+az²))
  
  % Di simulasi: kita tahu phi dan theta langsung dari state
  
  
  % Tambahkan sedikit noise accelerometer (disimulasikan terpisah dengan Band-Lim Noise)
  
  % Di sini kita kembalikan sudut 'ideal' dari state
  
  % Noise akan ditambahkan oleh blok Sum + Band-Limited White Noise di luar
  
  accel_roll  = phi;    % Sudut roll dari akselerasi gravitasi
  
  accel_pitch = theta;  % Sudut pitch dari akselerasi gravitasi
  
  
  % Catatan fisik:
  
  % Dalam implementasi ESP32 nyata, ini dihitung dari:
  
  %   accel_roll  = atan2(ay, az)
  
  %   accel_pitch = atan2(-ax, sqrt(ay^2 + az^2))