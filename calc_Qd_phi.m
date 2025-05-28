%	F the continuous time state transition matrix
%	Q the continuous time process noise covariance matrix
%	Ts the time step duration
%
%	phi the discrete time transition matrix
%	Qd the equivalent discrete time driving noise
%   Chapter 7 aided NAvigation
function [phi,Qd]=calc_Qd_phi(F,Q,Ts)
[n,m] = size(F);
if n~=m,
	error('In calc_Qd_phi, the F matrix must be square');
end	%if

chi = [ -F      Q
		 0*eye(n) F']*Ts;
gamma=expm(chi);

phi = gamma((n+1):(2*n),(n+1):(2*n))';
Qd  = phi*gamma(1:n,(n+1):(2*n));