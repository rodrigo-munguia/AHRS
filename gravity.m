%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  funtion g=gravity(lambda,h) 
%
%> @brief Function for calculating the magnitude of the local gravity. 
%>
%> @details Function for calculation of the local gravity vector based 
%> upon the WGS84 gravity model. 
%>
%> @param[out]  g          magnitude of the local gravity vector [m/s^2] 
%> @param[in]   lambda     latitude [degrees] 
%> @param[in]   h          altitude [m]  
%>
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function g=gravity(lambda,h)

lambda=pi/180*lambda;
gamma=9.780327*(1+0.0053024*sin(lambda)^2-0.0000058*sin(2*lambda)^2);
g=gamma-((3.0877e-6)-(0.004e-6)*sin(lambda)^2)*h+(0.072e-12)*h^2;
end