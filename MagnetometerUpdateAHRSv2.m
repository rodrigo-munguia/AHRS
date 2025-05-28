function [x P x2 P2]= MagnetometerUpdateAHRSv2(u,x,P,x2,P2)
global GVAR;
global PAR;



 GVAR.T_m = GVAR.t;
 
 if PAR.DirectMethodActive == 1 
 
Rm = PAR.Rm; 

mb_t = u.mb_t;
                Rn2b    = quat2R(x(1:4,1));
                Rb2n    = Rn2b';
                mb_N = Rb2n*mb_t;
                mb_N(3) = 0;
                mb_N = mb_N/norm(mb_N );
                mb_B = Rn2b*mb_N;  % Magnetometer measurements with earth magnetic field component Z removed     
                
                head_m = atan2(-mb_B(2),mb_B(1));

               JHe   =  JacH6b(x);
               
               zh = atan2(2*(x(2)*x(3)- x(1)*x(4)) , 1 - 2*(x(3)^2 + x(4)^2) ); % predicted heading
              
               
               Ram = Rm*3;
              
               S = JHe*P*JHe' + Ram;  %Innovation matrix
               K   = P*JHe'/S;     % KF gain
               P  = P - K*(JHe*P);               % meas update for cov
            
               in =  head_m - zh;  % heading measured - predicted heading 
               %[in head_m  zh]
               xi = K*in;
               
               %xi(7) = 0;
               x(1:10) = x(1:10) + xi;
 else
     x = nan;
     P = nan;
     
 end;
 
  %**************************************************************************             
%**************************************************************************
% indirect method

if PAR.IndirectMethodActive == 1
    Rn2b    = quat2R(x2(1:4,1));
    Rb2n = Rn2b';
                     mb_t = u.mb_t;
                     Hm = zeros(1,9);  % Accelometer Measuremnt model initialization                  
                     Hm(1,3) = 1;                           
                    
                     mhatg_n =  Rb2n*mb_t;
                     mhatg_n(3) = 0;
                     mhatg_n = mhatg_n/norm(mhatg_n);                     
                   
                     Rm = PAR.Rm*3; 
                     
                    K   = P2*Hm'*inv(Hm*P2*Hm'+Rm);     % KF gain                   
                   
                    P2  = P2 - K*(Hm*P2);               % meas update for cov
                    
               
                   
                     ym = [1 0 0]';                     
                     
                     mres = ym - mhatg_n;                     
                    
                     res = mres;                     
                     
                     del_x = K*res(2);              % compute state correction
                                     
                     
                     rho = del_x(1:3);
                     rho_cross = [0     -rho(3) rho(2)
                                  rho(3) 0     -rho(1)
                                 -rho(2) rho(1) 0];
                             I = eye(3,3);
                     Rn2b = Rn2b*(I-rho_cross);
                     qpc = Rot2Quat(Rn2b); 
                     qpc = qpc/norm(qpc); 
                     x2(1:4)  = qpc ;          % correct quaternion
                   
                     x2(5:10) = x2(5:10) + del_x(4:9);    % correct state
                     



else
     x2 = nan;
     P2 = nan;
     
 end;  
    
    
    
    