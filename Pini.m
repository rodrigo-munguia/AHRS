function [P] = Pini(sigma_nu_a,sigma_nu_g,elapsed_time)
 global PAR;
  %  from  ahrs_dmu4 code:
  int_T = elapsed_time;  
  P = zeros(9,9);
  ge = 9.78;              % gravity magnitude
  Rm = (1*pi/180)^2;      % rad^2, equiv to 1 deg, magnetometer noise
  
  
  sigma_nu_g = sigma_nu_g*20;
  
  %sigma_xg =  PAR.sigma_xg*100;
  
  P(1:3,1:3) = (diag([sigma_nu_a/ge,sigma_nu_a/ge,sqrt(Rm)])^2)/int_T;   %(1*pi/180)^2*eye(3,3); % rho uncertainty, rad
  
  
  P(4:6,4:6) = (sigma_nu_g)^2*eye(3,3)/int_T; % gyro bias uncertainty, rad/s, 4d/hr=0.001 d/s
  %P(4:6,4:6) = eye(3,3)*(sigma_xg)^2;
  
  P(7:9,7:9) = 0*(2e-3)^2*eye(3,3); % accel bias uncertainty, m/s/s
  P(1,8) =  sqrt( P(1,1)*P(8,8) );
  P(2,9) =  sqrt( P(2,2)*P(9,9) );
  P(7:9,1:3) = P(1:3,7:9)';
  
   % NOTE: should be replaced by equation 10.66 from aided navigation