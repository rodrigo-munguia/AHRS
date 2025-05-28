% euler angles to matrix R_n_to_b   (naviagation to body) %2.43
function [R] = R_n_to_b(euler)

phi = euler(1);
theta = euler(2);
psi = euler(3);

R = [[          cos(psi)*cos(theta)                       sin(psi)*cos(theta)                            -sin(theta)        ];
            [-sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi) cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi) cos(theta)*sin(phi)];
            [sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi) -cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi) cos(theta)*cos(phi)]];