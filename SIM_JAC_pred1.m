clear all

%%%%%%%  Jacobianos 
%*******************************************************************
% df_dq are the derivatives of f respect to quaterion orientation

syms  q1 q2 q3 q4 w_x w_y w_z b_x b_y b_z  real
syms  a11 a12 a13 a14 a21 a22 a23 a24 a31 a32 a33 a34 a41 a42 a43 a44 real 

q = [q1 q2 q3 q4]';

A = [[a11 a12 a13 a14];
     [a21 a22 a23 a24];
     [a31 a32 a33 a34];
     [a41 a42 a43 a44]];
 
 
 qn = A*q;
 
 
 dfq_dq = jacobian( qn, [A])