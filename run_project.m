%% AE4311 Assignment 2 - Command Filtered Adaptive/Incremental Backstepping Flight Control
% Change MODE below then run.
%
%   'CABS_NOMINAL'      -> one nominal CABS run
%   'CABS_COMPARE'      -> perfect / wrong-off / wrong-on
%   'CIBS_NOMINAL'      -> one nominal CIBS run
%   'CIBS_ROBUSTNESS'   -> original uniform nominal and +/-10...60% study
%   'CIBS_NONUNIFORM'   -> nonuniform parameter uncertainty experiment
%   'REPORT_PLOTS'      -> create compact report figures
%   'ALL'               -> run CABS comparison + both CIBS robustness
%   experiments

clear;clc;
close all;
MODE = 'ALL';   %CHANGE THIS LINE ONLY
[P,C,E] = setup_project();
if ~exist(E.resultsDir,'dir')
    mkdir(E.resultsDir);
end %%i like to create results folder for my projects in order to save them automatically in there,
%%this is present in other running scripts as well, it could be removed
switch upper(MODE)
    %% nominal cabs / kinda useless
    case 'CABS_NOMINAL'
        opt.rho0 = C.CABS.rhoPerfect;
        opt.adaptationOn = false;
        R = simulate_controller('CABS',P,C,opt);
        plot_dashboard('single',R,P,E);
        disp(struct2table(R.metrics,'AsArray',true));
        save(fullfile(E.resultsDir,'CABS_nominal.mat'),'R','P','C');
    %% cabs comparison
    case 'CABS_COMPARE'
        CABS = run_cabs_comparison(P,C);
        plot_dashboard('cabs_compare',CABS,P,E);
        print_cabs_table(CABS);
        save(fullfile(E.resultsDir,'CABS_comparison.mat'),'CABS','P','C');
    %% nominal cibs
    case 'CIBS_NOMINAL'
        P.thetaPlant = P.thetaTrue;
        R = simulate_controller('CIBS',P,C);
        plot_dashboard('single',R,P,E);
        disp(struct2table(R.metrics,'AsArray',true));
        save(fullfile(E.resultsDir,'CIBS_nominal.mat'),'R','P','C');
    %% cibs uniform robustness simulation
    case 'CIBS_ROBUSTNESS'
        CIBS = run_cibs_robustness(P,C,E.CIBS.uncertaintyLevels);
        plot_dashboard('cibs_robustness',CIBS,P,E);
        print_cibs_table(CIBS);
        save(fullfile(E.resultsDir,'CIBS_robustness.mat'),'CIBS','P','C');
    %% cibs nonuniform simulation
    case 'CIBS_NONUNIFORM'
        CIBS_NU = run_cibs_nonuniform(P,C,E.CIBS.nonuniformLevels,E.CIBS.nonuniformPatterns);
        print_cibs_nonuniform_table(CIBS_NU);
        plot_cibs_nonuniform_study(CIBS_NU,P,C,E.resultsDir);
        save(fullfile(E.resultsDir,'CIBS_nonuniform.mat'),'CIBS_NU','P','C','E');
    %% plots
    case 'REPORT_PLOTS'

        A = load(fullfile(E.resultsDir,'CABS_comparison.mat'));
        B = load(fullfile(E.resultsDir,'CIBS_robustness.mat'));
        plot_report_figures(A.CABS,B.CIBS,P,C,E);
    %% run everything
    case 'ALL'
        %% cabs
        CABS = run_cabs_comparison(P,C);
        print_cabs_table(CABS);
        save(fullfile(E.resultsDir,'CABS_comparison.mat'),'CABS','P','C');
        plot_dashboard('cabs_compare',CABS,P,E);
        %% uniform CIBS robustness
        CIBS = run_cibs_robustness(P,C,E.CIBS.uncertaintyLevels);
        print_cibs_table(CIBS);
        save(fullfile(E.resultsDir,'CIBS_robustness.mat'),'CIBS','P','C');
        plot_dashboard('cibs_robustness',CIBS,P,E);
        %% nonuniform CIBS robustness
        CIBS_NU = run_cibs_nonuniform(P,C,E.CIBS.nonuniformLevels,E.CIBS.nonuniformPatterns);
        print_cibs_nonuniform_table(CIBS_NU);
        save(fullfile(E.resultsDir,'CIBS_nonuniform.mat'),'CIBS_NU','P','C','E');
        plot_cibs_nonuniform_study(CIBS_NU,P,C,E.resultsDir);
        %% report figures
        plot_report_figures(CABS,CIBS,P,C,E);
end

%% local experiment helper functions
function D = run_cabs_comparison(P,C)
opt.rho0 = C.CABS.rhoPerfect;
opt.adaptationOn = false;
D.perfect = simulate_controller('CABS',P,C,opt);
opt.rho0 = C.CABS.rhoWrong;
opt.adaptationOn = false;
D.wrongOff = simulate_controller('CABS',P,C,opt);
opt.adaptationOn = true;
D.wrongOn = simulate_controller('CABS',P,C,opt);
end

%% uniform cibs robustnes
function D = run_cibs_robustness(P,C,levels)
P.thetaPlant = P.thetaTrue;
D.nominal = simulate_controller('CIBS',P,C);
D.levels = levels;
D.minus = cell(numel(levels),1);
D.plus = cell(numel(levels),1);
for i = 1:numel(levels)
    %% negative uniform uncertainty
    Pm = P;
    Pm.thetaPlant = (1-levels(i))*P.thetaTrue;
    D.minus{i} = simulate_controller('CIBS',Pm,C);
    %% positive uniform uncertainty
    Pp = P;
    Pp.thetaPlant = (1+levels(i))*P.thetaTrue;
    D.plus{i} = simulate_controller('CIBS',Pp,C);
end
end

%% nonuniform cibs robustness
function D = run_cibs_nonuniform(P,C,levels,patterns)
%% nominal case
Pn = P;
Pn.thetaPlant = P.thetaTrue;
D.nominal = simulate_controller('CIBS',Pn,C);
D.levels = levels;
D.patterns = patterns;
%% controller assumed effectiveness
D.gCtrl = P.Kq*(C.CIBS.thetaCtrl(6)*P.M+C.CIBS.thetaCtrl(7));
%% experiment families
familyNames = {'drift','control','full'};
for f = 1:numel(familyNames)
    family = familyNames{f};
    D.(family) = cell(numel(levels),1);
    direction = patterns.(family);
    for i = 1:numel(levels)
        level = levels(i);
        %% parameter-error vector
        epsVec = level*direction;
        %% construct perturbed physical parameters
        Pcase = P;
        Pcase.thetaPlant = P.thetaTrue.*(1+epsVec);
        %% running simulation
        R = simulate_controller('CIBS',Pcase,C);
        R.epsilonVector = epsVec;
        R.thetaPlant = Pcase.thetaPlant;
        R.gPlant = P.Kq*(Pcase.thetaPlant(6)*P.M+Pcase.thetaPlant(7));
        R.gCtrl = D.gCtrl;
        R.gRatio = R.gPlant/D.gCtrl;
        D.(family){i} = R;
    end
end

end

%% printing cabs table

function print_cabs_table(D)
Case = [ ...
    "Perfect";
    "Wrong / Adapt OFF";
    "Wrong / Adapt ON"];
R = { ...
    D.perfect;
    D.wrongOff;
    D.wrongOn};
T = metric_table(Case,R);
fprintf('CABS PERFORMANCE COMPARISON\n');
disp(T);
end

%% printing uniform cibs test

function print_cibs_table(D)
Case = "Nominal";
R = {D.nominal};
for i = 1:numel(D.levels)
    p = round(100*D.levels(i));
    Case(end+1,1) = "-" + string(p) + "%";
    R{end+1,1} = D.minus{i};
    Case(end+1,1) = "+" + string(p) + "%";
    R{end+1,1} = D.plus{i};
end
T = metric_table(Case,R);
fprintf('CIBS UNIFORM ROBUSTNESS COMPARISON\n');
disp(T);
end

%% printing nonuniform cibs table

function print_cibs_nonuniform_table(D)
Case = "Nominal";
R = {D.nominal};
gPlant = D.gCtrl;
gRatio = 1;
familyNames = {'drift','control','full'};
familyLabels = { ...
    'Drift only', ...
    'Control effectiveness only', ...
    'Full nonuniform'};

for f = 1:numel(familyNames)
    family = familyNames{f};
    for i = 1:numel(D.levels)
        p = round(100*D.levels(i));
        Case(end+1,1) = string(sprintf('%s - max %d%%',familyLabels{f},p));
        R{end+1,1} = D.(family){i};
        gPlant(end+1,1) = D.(family){i}.gPlant;
        gRatio(end+1,1) = D.(family){i}.gRatio;
    end
end

T = metric_table(Case,R);
T.gPlant = gPlant;
T.gPlant_over_gCtrl = gRatio;
fprintf('CIBS NONUNIFORM ROBUSTNESS COMPARISON\n');
disp(T);

%% printing actual perturbation vectors, only for visualising
fprintf('NONUNIFORM PARAMETER ERROR VECTORS\n');

for f = 1:numel(familyNames)
    family = familyNames{f};
    fprintf('\n%s\n',familyLabels{f});
    for i = 1:numel(D.levels)
        p = round(100*D.levels(i));
        epsPct = 100*D.(family){i}.epsilonVector;
        fprintf('Max level %d%%:\n',p);
        fprintf(['theta1 %+6.1f%% | theta2 %+6.1f%% | theta3 %+6.1f%% | ', ...
                 'theta4 %+6.1f%% | theta5 %+6.1f%% | theta6 %+6.1f%% | ', ...
                 'theta7 %+6.1f%%\n'], ...
                 epsPct(1),epsPct(2),epsPct(3),epsPct(4), ...
                 epsPct(5),epsPct(6),epsPct(7));
    end
end

end

%% generic metric table

function T = metric_table(Case,R)

n = numel(R);
a = zeros(n,1);
am = zeros(n,1);
q = zeros(n,1);
qm = zeros(n,1);
d = zeros(n,1);
dm = zeros(n,1);
sat = zeros(n,1);

for i = 1:n
    a(i) = R{i}.metrics.RMSE_alpha_deg;
    am(i) = R{i}.metrics.maxAbs_alpha_deg;
    q(i) = R{i}.metrics.RMSE_q_deg_s;
    qm(i) = R{i}.metrics.maxAbs_q_error_deg_s;
    d(i) = R{i}.metrics.deltaRMS_deg;
    dm(i) = R{i}.metrics.maxAbs_delta_deg;
    sat(i) = R{i}.metrics.saturationPercent;
end

T = table(Case,a,am,q,qm,d,dm,sat, ...
    'VariableNames',{ ...
    'Case', ...
    'RMSE_alpha_deg', ...
    'MaxAlphaErr_deg', ...
    'RMSE_q_deg_s', ...
    'MaxQErr_deg_s', ...
    'DeltaRMS_deg', ...
    'MaxDelta_deg', ...
    'Saturation_pct'});

end