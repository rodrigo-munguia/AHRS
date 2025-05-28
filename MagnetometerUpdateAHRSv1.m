function [x P]= MagnetometerUpdateAHRSv1(u,x,P)
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