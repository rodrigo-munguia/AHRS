% ********************************************************************
% Load data 
%  
%*********************************************************************

global DATA;
global PAR;

if PAR.DataO == '3DM'

%*****************************************************************
% 3DM-GX3-45 data
%
  DATA3DMGX345;
%*****************************************************************
% Malaga Data set
%
elseif PAR.DataO == 'Mal'
    
 DATAmalaga;   
  
%*****************************************************************
elseif PAR.DataO == 'RaD'
% ********************************************************************
% Load data captured from a Rawseeds datasets proyect
    
    
  DATArawseeds2;  
    
end;