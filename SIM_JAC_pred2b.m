clear all

%%%%%%%  Jacobianos 
%*******************************************************************
% df_dq are the derivatives of f respect to quaterion orientation

syms  w  real

syms W1 W2 W3 real

syms w_x w_y w_z delta_t real
 
w_u = [w_x w_y w_z];
%{
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
%}

dMc = cos(w)*eye(4,4);

dMc_dw  = jacobian( dMc, [w])'


dw   =  sqrt(W1^2 + W2^2 + W3^2) ;

dw_dW  = jacobian( dw, [W1 W2 W3])

dW   = w_u*delta_t/2;

dw_dw_u  = jacobian( dW , [w_u])


