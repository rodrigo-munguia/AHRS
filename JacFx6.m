function [JFx, JFu]   =  JacFx6(x,w_u,delta_t,Gbias,lambda_w,lambda_g)
% Munguia 2010
%
% x meaning
               % index  1  2  3  4  5   6   7   8  9   10
               %       q1 q2 q3 q4 w_x w_y w_z b_x b_y b_z     
               %   [q1 q2 q3 q4] -> quaternion orientation  
               %   [w_x w_y w_z ] -> vel rotation in the body frame
               %   [b_x b_y b_z ] -> gyro bias
% jacobian of f respect to state x
% where f = qprod(qWR,v2q(Rb2n*(wW*delta_t))
% f = quatmultiply(q,p);
% q -> state quaternion
% p = v2q(quat2R(q)*(wW*delta_t)
% p -> quaternion representing rotation produced by giro inputs
% quatmultiply -> quaternion product
% v2q -> rotation vector to quaternion.
% Rb2n -> current rotation matrix body to navigation grame
%  Rb2n = quat2R(x(1:4,1));
% quat2R -> quaternion to rotation matrix
% wW = -((w_u)-Gbias);  giro inputs after bias correction 

% ***  Jacobians are formed by
% JFx = [dfq_dq dfq_dw     0    ]    
%       [  0    dfw_du   dfw_db ]
%       [  0      0      dfb_db ]
% dfq_dq are the derivateves of f (quaternion part) respect to quaternion q 
% dfq_dw are the derivatives of f (quaternion part)respect to velocity of rotation.
% dfw_du are the derivatives of f (velocity part)respect to itself.
% dfw_db are the derivatives of f (velocity part)respect to gyro bias.
% dfb_db are the derivatives of f (gyro bias part) respect to itself
% and:
% JFu = [  0        ]
%       [  dfw_dwu  ]
%       [  0        ]
% df_du are the derivatives of f respect to input giros w_u

JFx = zeros(4,4);



q = x(1:4,1); %
%Gbias = x(5:7,1);  %

wb = -((w_u)-Gbias)*delta_t;  % Giro compensated readings

Rn2b    = quat2R(x(1:4,1));
Rb2n    = Rn2b';

ww = Rb2n*(wb);
p= v2q(ww);

% ****************************************************************
% df_dq are the derivatives of f respect to quaterion orientation
% z = f(q,p) where p = g(q)  then by the chain rule:
% dz_dq = dz_dq' + dz_dp*dp_dq   therefore:
% dfq_dq =  df_dq1 + (df_dp * dp_dww * dww_dq )   

df_dq1 = [[  p(1), -p(2), -p(3), -p(4)]
         [  p(2),  p(1),  p(4), -p(3)]
         [  p(3), -p(4),  p(1),  p(2)]
         [  p(4),  p(3), -p(2),  p(1)]];         

   
%*****************************************************************

 df_dp = [[  q(1), -q(2), -q(3), -q(4)]
         [  q(2),  q(1), -q(4),  q(3)]
         [  q(3),  q(4),  q(1), -q(2)]
         [  q(4), -q(3),  q(2),  q(1)]];

% dp_dww =  derivatives of function v2q respect to ww

theta=norm(ww);
v_n=ww/norm(ww);
ww1 =ww(1);
ww2 = ww(2);
ww3 = ww(3);
% 
% q=[cos(theta/2)  sin(theta/2)*v_n'];
%tv2q1 =  cos(theta/2);
%tv2q2 = sin(theta/2)*v_n';
%tv2q =  [tv2q1  tv2q2];

dtv2q1_dtheta =   -1/2*sin(1/2*theta);
%dtv2q1_dv_n = 0;
dtv2q2_dtheta = 1/2*cos(1/2*theta)*v_n;
dtv2q2_dv_n =  sin(1/2*theta);

dtheta_dww = [ ww1/norm(ww), ww2/norm(ww), ww3/norm(ww)];

ww_quad = ww1^2+ww2^2+ww3^2;

dtv_n_dww = [[ (ww2^2+ww3^2)/(ww_quad )^(3/2),      -ww1/(ww_quad )^(3/2)*ww2,      -ww1/(ww_quad )^(3/2)*ww3]
            [      -ww1/(ww_quad )^(3/2)*ww2, (ww1^2+ww3^2)/(ww_quad )^(3/2),      -ww2/(ww_quad )^(3/2)*ww3]
            [      -ww1/(ww_quad )^(3/2)*ww3,      -ww2/(ww_quad )^(3/2)*ww3, (ww1^2+ww2^2)/(ww_quad )^(3/2)]];

dp_dww = [[dtv2q1_dtheta*dtheta_dww]
          [dtv2q2_dtheta*dtheta_dww +  dtv2q2_dv_n*dtv_n_dww    ]];
      
       
%***************************************************
% dww_dq = [ dR_dq1 dR_dq2 dR_dq3 dR_dq4];

dR_dq1 =[[  2*q(1),  2*q(4), -2*q(3)]
         [ -2*q(4),  2*q(1),  2*q(2)]
         [  2*q(3), -2*q(2),  2*q(1)]]*wb;
     
dR_dq2 = [[  2*q(2),  2*q(3),  2*q(4)]
          [  2*q(3), -2*q(2),  2*q(1)]
          [  2*q(4), -2*q(1), -2*q(2)]]*wb;

dR_dq3 = [[ -2*q(3),  2*q(2), -2*q(1)]
          [  2*q(2),  2*q(3),  2*q(4)]
          [  2*q(1),  2*q(4), -2*q(3)]]*wb;

dR_dq4 = [[ -2*q(4),  2*q(1),  2*q(2)]
          [ -2*q(1), -2*q(4),  2*q(3)]
          [  2*q(2),  2*q(3),  2*q(4)]]*wb;      

dww_dq = [ dR_dq1 dR_dq2 dR_dq3 dR_dq4];

%****************************************
dfq_dq =  df_dq1 + df_dp * dp_dww * dww_dq ; % dfq_dq are the derivateves of f (quaternion part) respect to quaternion q 

%****************************************


%****************************************
% dfq_dw are the derivatives of f (quaternion part)respect to velocity of rotation.
% dfq_dw = df_dp * dp_dww * dww_dw


dR_dwb1 =   [ delta_t*(q(1)^2 + q(2)^2 - q(3)^2 - q(4)^2)
              -delta_t*(2*q(1)*q(4) - 2*q(2)*q(3))
              delta_t*(2*q(1)*q(3) + 2*q(2)*q(4))];

dR_dwb2 = [ delta_t*(2*q(1)*q(4) + 2*q(2)*q(3))
            delta_t*(q(1)^2 - q(2)^2 + q(3)^2 - q(4)^2)
            -delta_t*(2*q(1)*q(2) - 2*q(3)*q(4))];
        
dR_dwb3 = [ -delta_t*(2*q(1)*q(3) - 2*q(2)*q(4))
             delta_t*(2*q(1)*q(2) + 2*q(3)*q(4))
             delta_t*(q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2) ];
         
dww_dw  = [dR_dwb1 dR_dwb2 dR_dwb3];         


%**************************************
dfq_dw = df_dp * dp_dww * dww_dw;
%**************************************

%**************************************
% dfw_dw are the derivatives of f (velocity part)respect to velocity of rotation.



  dfw_dw  =        [ [(1-(lambda_w*delta_t)) 0  0 ]
                    [0 (1-(lambda_w*delta_t)) 0  ]
                    [0  0  (1-(lambda_w*delta_t))] ];
                
                dfw_dw = zeros(3,3);
                
                
% dfw_db are the derivatives of f (velocity part)respect to gyro bias.

dfw_db = [[1 0   0]
          [0   1 0]
          [0    0  1]];
% dfb_db are the derivatives of f (gyro bias part) respect to itself      
      
dfb_db =       [ [(1-(lambda_g*delta_t)) 0  0 ]
                 [0 (1-(lambda_g*delta_t)) 0  ]
                 [0  0  (1-(lambda_g*delta_t))] ];

%**************************************************************************
% JFx = [dfq_dq dfq_dw     0    ]    
%       [  0    dfw_dw   dfw_db ]
%       [  0      0      dfb_db ]
%**************************************************************************

JFx = zeros (10,10);
JFx(1:4,1:4) = dfq_dq;
JFx(1:4,5:7)  = dfq_dw;
JFx(5:7,5:7) = dfw_dw ;
JFx(5:7,8:10) = dfw_db;
JFx(8:10,8:10) = dfb_db;



%**************************************************************************

% JFu = [  0         0]
%       [  dfw_dwu   0]
%       [  0         dfw_db]

JFu  = zeros (10,6);

JFu(5:7,1:3) =   -1*eye(3);
JFu(8:10,4:6) =  1*eye(3);




%{
% df_xg are the derivatives of f respect to giro bias xg
% df_xg = df_dp * dp_dww * dww_dxg


dww_db1 = [(2*q(2)*q(3)+2*q(1)*q(4))*delta_t
       (q(1)^2-q(2)^2+q(3)^2-q(4)^2)*delta_t
        (2*q(3)*q(4)-2*q(1)*q(2))*delta_t];

dww_db2 = [(2*q(2)*q(3)+2*q(1)*q(4))*delta_t
           (q(1)^2-q(2)^2+q(3)^2-q(4)^2)*delta_t
            (2*q(3)*q(4)-2*q(1)*q(2))*delta_t];

dww_db3 =  [(2*q(2)*q(4)-2*q(1)*q(3))*delta_t
            (2*q(1)*q(2)+2*q(3)*q(4))*delta_t
            (q(1)^2-q(2)^2-q(3)^2+q(4)^2)*delta_t];

dww_dxg = [dww_db1 dww_db2 dww_db3];

%*****************************************************************
df_xg = df_dp * dp_dww * dww_dxg;

%*****************************************************************     

%JFx(1:4,1:7) = [df_dq df_xg];
%JFx(5:7,5:7) = eye(3,3);
%JFx = df_dq; 

%******************************************************************
% df_du = df_dp * dp_dww * dww_dwb

JFu =  zeros(4,6);

dww_du = -dww_dxg;

df_du = df_dp * dp_dww * dww_du;

JFu(1:4,1:3) = df_du;
JFu(1:4,4:6) = df_xg; 
%JFu(5:7,4:6) = eye(3,3);
     
 
qqq = 10;
     
 %************************************************************************
 %}