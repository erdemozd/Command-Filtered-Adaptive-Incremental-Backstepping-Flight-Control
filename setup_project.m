function [P,C,E] = setup_project()
%SETUP_PROJECT configuration for the complete project.
%% Aircraft / simulation parameters
P.g     = 32.2;
P.W     = 450;
P.VT    = 3109.3;
P.Iyy   = 182.5;
P.qbar  = 6132.8;
P.S     = 0.44;
P.cbar  = 0.75;
P.M     = 2.2;
P.m     = P.W/P.g;
P.Ts    = 0.01;
P.tEnd  = 60;
P.alphaRefAmp = deg2rad(11);
P.omegaRef    = 0.6;
P.deltaMax = deg2rad(9.5);
P.deltaMin = -P.deltaMax;
P.Kq = P.qbar*P.S*P.cbar/P.Iyy;

%% True pitch-dynamics parameters
P.thetaTrue = [ ...
     303.1;
    -246.3;
     -37.56;
      71.51;
      10.01;
      12.0393;
      48.2246];
% Actual nonlinear plant parameters.
P.thetaPlant = P.thetaTrue;
%% Initial conditions
P.alpha0 = 0;
P.q0     = 0;
P.delta0 = 0;
%% Common backstepping / command-filter settings
C.c1 = 4.0;
C.c2 = 8.0;
C.wn   = 20;
C.zeta = 0.7;

A = [0 1;
    -C.wn^2 -2*C.zeta*C.wn];

B = [0;
     C.wn^2];

Aug = expm([A B; zeros(1,3)]*P.Ts);

C.Ad_cf = Aug(1:2,1:2);
C.Bd_cf = Aug(1:2,3);
C.chi10 = 0;
C.gHatMin = 20;
%% CABS SETTINGS
C.CABS.thetaNom = P.thetaTrue;
C.CABS.rhoPerfect = ones(7,1);
C.CABS.rhoWrong = [ ...
    0.80;
    1.20;
    0.75;
    1.25;
    0.85;
    1.15;
    0.80];
C.CABS.Gamma = diag([ ...
    0.02 ...
    0.02 ...
    0.02 ...
    0.02 ...
    0.02 ...
    0.005 ...
    0.005]);
C.CABS.rhoMin = 0.4*ones(7,1);
C.CABS.rhoMax = 1.6*ones(7,1);
C.CABS.freezeOnSaturation = true;
%% CIBS SETTINGS
% controller always retains nominal control-effectiveness info
% during the robustness exp.
C.CIBS.thetaCtrl = P.thetaTrue;
%% EXPERIMENT DEFINITIONS
% original uniform robustness experiment
E.CIBS.uncertaintyLevels = ...
    [0.10 0.20 0.30 0.40 0.50 0.60];
%% Nonuniform robustness study
E.CIBS.nonuniformLevels = [0.10 0.20 0.30]
% case 1: Aerodynamic drift uncertainty only
% theta1 ... theta5 are perturbed independently.
% theta6 and theta7 remain exact.
E.CIBS.nonuniformPatterns.drift = [ ...
     1.00;
    -0.70;
     0.40;
    -1.00;
     0.80;
     0.00;
     0.00];
% case 2: Control-effectiveness uncertainty only
% Only theta6 and theta7 are changed.
E.CIBS.nonuniformPatterns.control = [ ...
     0.00;
     0.00;
     0.00;
     0.00;
     0.00;
    -1.00;
    -0.40];
% case 3: Full nonuniform uncertainty
% All seven pitch parameters are perturbed differently.
E.CIBS.nonuniformPatterns.full = [ ...
     1.00;
    -0.70;
     0.40;
    -1.00;
     0.80;
    -0.60;
    -0.30];
%% Results directory
E.resultsDir = 'results';
end