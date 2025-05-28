function [JH]   =  JacH6a(x,delta_t,eT,g,Gbias)
% x -> state
% delta_t ->  sample period
% eT -> elapsed time since last update
% Jacobian h
%*****  Accelerometer Measurement Model
%  z = h(x) 
%  z = R(q)n_to_b * R(xg,t)n_to_b * [0 0 g]' ;
%  JH = [dh_dq dh_dxg]

q = x(1:4,1); %
xg = Gbias;  %

phix = xg(1);
thetax = xg(2);
psix = xg(3);

%eT = 0; % temp....

Rq_n2b = quat2R(q);
Rxg_n2b =  Euler_To_R_a_to_b([phix thetax psix]*eT);

%w = Rxg_n2b*[0 0 g]';


% ***  dh_dq


w = [0 0 g]';

dR_dq1 = [[  2*q(1), -2*q(4),  2*q(3)]
          [  2*q(4),  2*q(1), -2*q(2)]
          [ -2*q(3),  2*q(2),  2*q(1)]]*w;     

dR_dq2 = [[  2*q(2),  2*q(3),  2*q(4)]
          [  2*q(3), -2*q(2), -2*q(1)]
          [  2*q(4),  2*q(1), -2*q(2)]]*w;
      
dR_dq3 = [[ -2*q(3),  2*q(2),  2*q(1)]
          [  2*q(2),  2*q(3),  2*q(4)]
          [ -2*q(1),  2*q(4), -2*q(3)]]*w;

dR_dq4 = [[ -2*q(4), -2*q(1),  2*q(2)]
          [  2*q(1), -2*q(4),  2*q(3)]
          [  2*q(2),  2*q(3),  2*q(4)]]*w;      

dh_dq = [ dR_dq1 dR_dq2 dR_dq3 dR_dq4];


  
JHa = dh_dq;
  
%JH = zeros(6,7);
JH = zeros(3,10);
JH(1:3,1:4) = JHa;
  
% JH = [[dh_dq  Rq_n2b*eT ]
%      [dh_dq2 Rq_n2b*eT]];

q = 10;
  
  



