% ********************************************************************
% Initialize Global Variables 
%*********************************************************************

global GVAR;

% Variables Initialization **********************************************
GVAR.fcnt  = 1;
GVAR.e = 0; % initial state
GVAR.t = 0; %
GVAR.T_a = 0; % accel update time base
GVAR.T_m = 0; % Magnetometer update time base
GVAR.T_v = 0; % Zero vel update time base
%T_b = t; % magnetometers update time time base

GVAR.int_g = 0;
GVAR.int_b = 0;
GVAR.int_w = 0;
GVAR.int_T  = 0;
GVAR.int_GC = 0;
GVAR.int_ini  = 0;
GVAR.init_Gbias = 0;
GVAR.UptateDone = 0;

GVAR.GC = 0;

GVAR.kgps = 1;
GVAR.kimu = 1;


%***********************************
% Indirect Method

GVAR.Phi = eye(9,9);
 GVAR.Qd  = zeros(9,9);