clear all

%%%%%%%  Jacobianos 
%*******************************************************************
% df_dq are the derivatives of f respect to quaterion orientation

syms  q1 q2 q3 q4 w_x w_y w_z b_x b_y b_z  real
syms delta_t real
 
w_u = [w_x w_y w_z];
q = [q1 q2 q3 q4]';

 
W   = w_u*delta_t/2;

w   =  sqrt(W(1)^2 + W(2)^2 + W(3)^2) ;


sinwow = sin(w)/w;

W_mat = [0  -W(1) -W(2) -W(3)
         W(1)  0   -W(3)  W(2)
         W(2) W(3)   0   -W(1)
         W(3) -W(2) W(1)   0  ];
                    
                
 qp = (cos(w)*eye(4,4) + W_mat*sinwow)*q;
 
 
 dfq_dw  = jacobian( qp, [w_u])