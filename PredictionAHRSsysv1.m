function [x P]= PredictionAHRSsysv1(u,x,P)
global PAR;
global GVAR;


delta_t = PAR.delta_t;
 GiroScale = PAR.GiroScale;
 sigma_nu_g =  PAR.sigma_nu_g ;
 sigma_xg = PAR.sigma_xg ;
 lambda_g  = PAR.lambda_g; 
 lambda_w = PAR.lambda_w; 
             
                      
           
                         
            %w_bn_b = -(((w_u))- x_gb);  % Giro compensated readings
            
            w_u  = u.w_bn_b*GiroScale + PAR.ExtraBias;   
              
              qWR = x(1:4,1);
            
             Rn2b    = quat2R(x(1:4,1));
             Rb2n    = Rn2b';
             
          
             %*************  Mechanization Model **************************
             %**   x_k+1 = f(x_k,u_k)  ********************************
            
             
                
             
             Fw = [ [(1-(lambda_w*delta_t)) 0  0     1    0      0    ]
                    [0 (1-(lambda_w*delta_t))  0       0     1   0    ]
                    [0  0  (1-(lambda_w*delta_t))      0        0   1 ]
                    [0  0           0          (1-(lambda_g*delta_t)) 0  0  ]
                    [0  0           0             0 (1-(lambda_g*delta_t)) 0]
                    [0  0           0             0  0 (1-(lambda_g*delta_t))]];
                
             Bw = [[-1    0      0   ]
                   [ 0     -1   0   ]
                   [ 0        0   -1 ]
                   [ 0        0        0   ]
                   [ 0        0        0   ]
                   [ 0        0        0   ]];
               
               %x(8:10) = [0 0 0]';
                
                
             x(5:10,1) = Fw*x(5:10,1) + Bw*w_u;  
             
             qp = reshape(qprod(qWR,v2q(Rb2n*(x(5:7)*delta_t))),4,1);
             qp = qp/norm(qp);
             x(1:4,1) = qp;  
             
             
             
               xb = x(8:10);
             
             %**********************************************************
             %  Jacobians
             [JFx, JFu]  =  JacFx6(x,w_u,delta_t,xb,lambda_w,lambda_g);             
             
             % ****  % Procces noise covariance matrix U:
             % sigma_nu_g -> noise angle giro drift
             % sigma_xg >  noise bias giro drift            
                     
             
             U = [ eye(3,3)*(sigma_nu_g)^2   zeros(3,3)
                          zeros(3,3)     eye(3,3)*(sigma_xg)^2 ];
             
             %***************************************************
             
             %***************************************************
             
             P = JFx*P*JFx' + JFu*U*JFu';    % Covariance matrix prediction     
              
             %****************************************************      