function [R_a_to_b] = Euler_To_R_a_to_b(euler)

% euler angles to rotation matrix R_a_to_b    % Aided Navigation 2.43

phi = euler(1);
 theta = euler(2);
 psi = euler(3);

R_a_to_b = [[          cos(psi)*cos(theta)                       sin(psi)*cos(theta)                            -sin(theta)        ];
            [-sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi) cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi) cos(theta)*sin(phi)];
            [sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi) -cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi) cos(theta)*cos(phi)]];