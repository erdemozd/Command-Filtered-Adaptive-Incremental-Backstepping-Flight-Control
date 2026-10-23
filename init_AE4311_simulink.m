%% AE4311 CABS / CIBS Simulink Initialization
% Run this script before simulating AE4311_CABS_CIBS.slx.
% It places all workspace variables required by the Simulink model into the
% MATLAB base workspace.
%
% Select one of the predefined cases below:
%   'CABS_NOMINAL'
%   'CABS_WRONG_OFF'
%   'CABS_WRONG_ON'
%   'CIBS_NOMINAL'
%clearvars -except ans
%% Select experiment
simCase = 'CABS_WRONG_ON';
%% Aircraft parameters
g     = 32.2;
W     = 450;
VT    = 3109.3;
Iyy   = 182.5;
qbar  = 6132.8;
S     = 0.44;
cbar  = 0.75;
M     = 2.2;
m  = W/g;
Kq = qbar*S*cbar/Iyy;
%% Simulation settings
Ts   = 0.01;
tEnd = 60;
alphaRefAmp = 11*pi/180;
omegaRef    = 0.6;
deltaMax = 9.5*pi/180;
deltaMin = -deltaMax;
%% Controller settings
c1   = 4;
c2   = 8;
wn   = 20;
zeta = 0.7;
gHatMin = 20;
%% True aerodynamic pitch-dynamics parameters
thetaTrue = [ ...
     303.1;
    -246.3;
     -37.56;
      71.51;
      10.01;
      12.0393;
      48.2246];
%% CABS settings
rhoPerfect = ones(7,1);
rhoWrong = [ ...
    0.80;
    1.20;
    0.75;
    1.25;
    0.85;
    1.15;
    0.80];

GammaCABS = diag([ ...
    0.02, ...
    0.02, ...
    0.02, ...
    0.02, ...
    0.02, ...
    0.005, ...
    0.005]);

rhoMin = 0.4*ones(7,1);
rhoMax = 1.6*ones(7,1);
%% Variables required directly by the Simulink model
% controllerMode:
%   1 -> CABS
%   2 -> CIBS

% thetaPlantSim: actual parameters used by the nonlinear aircraft plant
% thetaCtrlSim: parameter vector assumed by the CIBS controller
% cabsRho0: initial normalized CABS parameter estimates

% cabsAdaptationOn:
%   0 -> adaptation OFF
%   1 -> adaptation ON

switch upper(simCase)
    case 'CABS_NOMINAL'
        controllerMode   = 1;
        thetaPlantSim    = thetaTrue;
        thetaCtrlSim     = thetaTrue;
        cabsRho0         = rhoPerfect;
        cabsAdaptationOn = 0;
    case 'CABS_WRONG_OFF'
        controllerMode   = 1;
        thetaPlantSim    = thetaTrue;
        thetaCtrlSim     = thetaTrue;
        cabsRho0         = rhoWrong;
        cabsAdaptationOn = 0;
    case 'CABS_WRONG_ON'
        controllerMode   = 1;
        thetaPlantSim    = thetaTrue;
        thetaCtrlSim     = thetaTrue;
        cabsRho0         = rhoWrong;
        cabsAdaptationOn = 1;
    case 'CIBS_NOMINAL'
        controllerMode   = 2;
        thetaPlantSim    = thetaTrue;
        thetaCtrlSim     = thetaTrue;
        cabsRho0         = rhoPerfect;
        cabsAdaptationOn = 0;
    case 'CIBS_CUSTOM'
        controllerMode = 2;
        % Example: uniform -20%% plant uncertainty.
        % Change epsilon to any desired value, e.g. +0.30 or -0.60.
        epsilon = -0.20;
        thetaPlantSim = (1 + epsilon)*thetaTrue;
        % CIBS controller retains the nominal parameter vector.
        thetaCtrlSim = thetaTrue;
        % CABS variables are inactive in CIBS mode, but are still defined
        % because their Constant blocks remain present in the model.
        cabsRho0         = rhoPerfect;
        cabsAdaptationOn = 0;
end
