% ********************************************************************
% Load data captured from a 3DM-GX3-45 unit
% Define parameters 
%*********************************************************************

global DATA;



datafile = PAR.DatasetName;


df = sprintf( 'data/%s.csv', datafile);
%rs = sprintf( 'exp\\Dir%d%s', FreqReads,datafile);

data = csvread(df,1,0); % Load all Data

DATA.IMU(:,2:10) = data(:,1:9);

DATA.GT.EULER(:,1:3) = data(:,10:12);


[m,n] = size(data);
PAR.nSteps = m;  