clear all

%%%%%%%  Jacobianos 
%*******************************************************************
% df_dq are the derivatives of f respect to quaterion orientation

syms  q1 q2 q3 q4  real

 
b = [q1 q2 q3 q4];

phi = atan(2*(b(3)*b(4)- b(1)*b(2))/1 - 2*(b(2)^2 + b(3)^2) ); %Aided Navigation %10.3
theta = asin(-2*( b(1)*b(3) + b(2)*b(4)) ); %  10.4
psi = atan(2*(b(2)*b(3)- b(1)*b(4)) / 1 - 2*(b(3)^2 + b(4)^2) ); % 10.5
 
 
    
    
    dfe_dq  = jacobian( [phi theta psi], [b])