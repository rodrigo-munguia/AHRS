function [x P]= AcelerometerUpdateAHRSv1(u,x,P)
global GVAR;
global PAR;

  GVAR.T_a =  GVAR.t;  
 %GC = PAR.g;
 GC = GVAR.GC;
 Ra = PAR.Ra;


 delta_t = PAR.delta_t;
  
  f_ip_p = -((u.f_ip_p*PAR.AccelScale)  ) ;                % m/s/s,   body frame specific force
  nrm_accel  = norm(f_ip_p) - GC; 
  
            JH   =  JacH6a(x,delta_t,0,GC,[0 0 0]); % measurement jacobian
                        
               % measurement noise matrix
            
               Raa = zeros(3,3);
              Raa(1:3,1:3) = (Ra + (1*nrm_accel)^2)*eye(3,3);    % account for acceleration as error
               
              Rn2b    = quat2R(x(1:4,1));
               
               S = JH*P*JH' + Raa;  %Innovation matrix
            
                K   = P*JH'/S;     % KF gain
               P  = P - K*(JH*P);               % meas update for cov
             % z = Rn2b*[0 0 Gc]';
               z = Rn2b*[0 0 GC]';
               
               % innovations
               in = f_ip_p - z;
              
               % state update
               xi = K*in;
               x(1:10) = x(1:10) + xi;
               
               
               