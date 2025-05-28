function [zupt T]=zero_velocity_detector2(u)


% Global struct holding the simulation settings
global PAR;

% Allocate memmory
zupt=zeros(1,length(u));


T=GLRT(u);


% Check if the test statistics T are below the detector threshold. If so, 
% chose the hypothesis that the system has zero velocity 
W=PAR.Window_size;
for k=1:length(T)
    if T(k)<PAR.gamma
       zupt(k:k+W-1)=ones(1,W); 
    end    
end

% Fix the edges of the detector statistics
T=[max(T)*ones(1,floor(W/2)) T max(T)*ones(1,floor(W/2))];
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  funtion T = GLRT(u)
%
%> @brief Function that runs the generalized likelihood test (SHOE detector). 
%>
%> @param[out]  T          The test statistics of the detector 
%> @param[in]   u          The IMU data vector.     
%>
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function T=GLRT(u)

global PAR;

g=PAR.g;
sigma2_a=PAR.sigma_a^2;
sigma2_g=PAR.sigma_g^2;
W=PAR.Window_size;


N=length(u);
T=zeros(1,N-W+1);

for k=1:N-W+1
   
    ya_m=mean(u(1:3,k:k+W-1),2);
    
    for l=k:k+W-1
        tmp=u(1:3,l)-g*ya_m/norm(ya_m);
        T(k)=T(k)+u(4:6,l)'*u(4:6,l)/sigma2_g+tmp'*tmp/sigma2_a;    
    end    
end

T=T./W;

end