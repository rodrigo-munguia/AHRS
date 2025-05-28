function [phi,theta,psi] = Quaternion_To_Euler(b)


    phi = atan2(2*(b(3)*b(4)- b(1)*b(2)),1 - 2*(b(2)^2 + b(3)^2) ); %Aided Navigation %10.3
    theta = asin(-2*( b(1)*b(3) + b(2)*b(4)) ); %  10.4
    psi = atan2(2*(b(2)*b(3)- b(1)*b(4)) , 1 - 2*(b(3)^2 + b(4)^2) ); % 10.5