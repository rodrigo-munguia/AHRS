function [q] = R_n_to_b_TO_Quat(R_n_to_b)

%   R_n_to_b  to Quaternion b  %D.15

b1 = (1/2)*sqrt( 1 + R_n_to_b(1,1)+ R_n_to_b(2,2)+ R_n_to_b(3,3) );
     
q = [   b1;
        (R_n_to_b(3,2)-R_n_to_b(2,3))/(4*b1);
        (R_n_to_b(1,3)-R_n_to_b(3,1))/(4*b1);
        (R_n_to_b(2,1)-R_n_to_b(1,2))/(4*b1)]; % D.15
% *************************