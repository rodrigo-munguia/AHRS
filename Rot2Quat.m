function [b]=Rot2Quat(R)
[U,S,V]=svd(R);
R = U*V';
if 1+R(1,1)+R(2,2)+R(3,3) > 0
    b(1,1)    = 0.5*sqrt(1+R(1,1)+R(2,2)+R(3,3));
    b(2,1)    = (R(3,2)-R(2,3))/4/b(1);
    b(3,1)    = (R(1,3)-R(3,1))/4/b(1);
    b(4,1)    = (R(2,1)-R(1,2))/4/b(1);
    b       = b/norm(b);    % renormalize
else
    R
    error('R diagonnal too negative.')
    b = zeros(4,1);
end