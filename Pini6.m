function [P] = Pini6(sigma_nu_a,sigma_nu_g,sigma_xg,elapsed_time,euler)

  %  from  ahrs_dmu4 code:
  int_T = elapsed_time;  
  P = zeros(10,10);
  ge = 9.78;              % gravity magnitude
  Rm = (1*pi/180)^2;      % rad^2, equiv to 1 deg, magnetometer noise
  
  
  % noise tuning for initial covariance matrix
  sigma_nu_a =  sigma_nu_a*.31;
  sigma_nu_g = sigma_nu_g*1;
  sigma_xg =  sigma_xg*100;
  %***********************************
  
  
  
  Pt = (diag([sigma_nu_a/ge,sigma_nu_a/ge,sqrt(Rm)])^2)/int_T;   %(1*pi/180)^2*eye(3,3); % rho uncertainty, rad
 

  phi = euler(1);
  theta = euler(2);
  psi = euler(3);
 
  % J is the jacobian of the euler to quaternation transformation.
J = [ [ -1/2*sin(1/2*phi)*cos(1/2*theta)*cos(1/2*psi)+1/2*cos(1/2*phi)*sin(1/2*theta)*sin(1/2*psi), -1/2*cos(1/2*phi)*sin(1/2*theta)*cos(1/2*psi)+1/2*sin(1/2*phi)*cos(1/2*theta)*sin(1/2*psi), -1/2*cos(1/2*phi)*cos(1/2*theta)*sin(1/2*psi)+1/2*sin(1/2*phi)*sin(1/2*theta)*cos(1/2*psi)]
[  1/2*cos(1/2*phi)*cos(1/2*theta)*cos(1/2*psi)+1/2*sin(1/2*phi)*sin(1/2*theta)*sin(1/2*psi), -1/2*sin(1/2*phi)*sin(1/2*theta)*cos(1/2*psi)-1/2*cos(1/2*phi)*cos(1/2*theta)*sin(1/2*psi), -1/2*sin(1/2*phi)*cos(1/2*theta)*sin(1/2*psi)-1/2*cos(1/2*phi)*sin(1/2*theta)*cos(1/2*psi)]
[ -1/2*sin(1/2*phi)*sin(1/2*theta)*cos(1/2*psi)+1/2*cos(1/2*phi)*cos(1/2*theta)*sin(1/2*psi),  1/2*cos(1/2*phi)*cos(1/2*theta)*cos(1/2*psi)-1/2*sin(1/2*phi)*sin(1/2*theta)*sin(1/2*psi), -1/2*cos(1/2*phi)*sin(1/2*theta)*sin(1/2*psi)+1/2*sin(1/2*phi)*cos(1/2*theta)*cos(1/2*psi)]
[ -1/2*sin(1/2*phi)*cos(1/2*theta)*sin(1/2*psi)-1/2*cos(1/2*phi)*sin(1/2*theta)*cos(1/2*psi), -1/2*cos(1/2*phi)*sin(1/2*theta)*sin(1/2*psi)-1/2*sin(1/2*phi)*cos(1/2*theta)*cos(1/2*psi),  1/2*cos(1/2*phi)*cos(1/2*theta)*cos(1/2*psi)+1/2*sin(1/2*phi)*sin(1/2*theta)*sin(1/2*psi)]];
 
  Pq = J*Pt*J';
  
  
  
  Pb  = (sigma_nu_g)^2*eye(3,3); % gyro bias uncertainty, rad/s, 4d/hr=0.001 d/s; 
 Pg = eye(3,3)*(sigma_xg)^2;
 Pg(3,3) = Pg(3,3)/10;
  
  
  P(1:4,1:4) = Pq;
  P(5:7,5:7) = Pb;
  P(8:10,8:10) = Pg;
  
  
  ww = 10;
  
  
   % NOTE: should be replaced by equation 10.66 from aided navigation