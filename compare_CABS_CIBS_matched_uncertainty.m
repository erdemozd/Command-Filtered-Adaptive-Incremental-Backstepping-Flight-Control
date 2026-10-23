%% AE4311 - Matched-Uncertainty Comparison of CABS and CIBS
% Both controllers are tested on exactly the same perturbed physical plant.
% Cases:
%   1. Nominal
%   2. Drift-only uncertainty
%   3. Control-effectiveness-only uncertainty
%   4. Full nonuniform uncertainty
% Levels:
%   10%, 20%, 30%
clear;clc;close all;
[P,C,E] = setup_project();
if ~exist(E.resultsDir,'dir')
    mkdir(E.resultsDir); %% see run_project.m - line 18
end
%% MATCHED UNCERTAINTY SETUP
levels = [0.10 0.20 0.30];
mainLevel = 0.30;
patterns(1).name = 'Drift only';
patterns(1).direction = [1;-0.7;0.4;-1;0.8;0;0];
patterns(2).name = 'Control effectiveness only';
patterns(2).direction = [0;0;0;0;0;-1;-0.4];
patterns(3).name = 'Full nonuniform';
patterns(3).direction = [1;-0.7;0.4;-1;0.8;-0.6;-0.3];
% CABS starts from the nominal model and adapts online.
cabsOptions.rho0 = C.CABS.rhoPerfect;
cabsOptions.adaptationOn = true;
% CIBS retains the nominal controller model.
C.CIBS.thetaCtrl = P.thetaTrue;
%% NOMINAL MATCHED BASELINE
Pnom = P;
Pnom.thetaPlant = P.thetaTrue;
fprintf('MATCHED CABS / CIBS COMPARISON\n');
fprintf('\nRunning nominal baseline...\n');
Nominal.CABS = simulate_controller('CABS',Pnom,C,cabsOptions);
Nominal.CIBS = simulate_controller('CIBS',Pnom,C);
%% RUN MATCHED UNCERTAINTY CASES
Results = struct();
gCtrl = P.Kq*(C.CIBS.thetaCtrl(6)*P.M+C.CIBS.thetaCtrl(7));
for ip = 1:numel(patterns)
    Results(ip).name = patterns(ip).name;
    Results(ip).direction = patterns(ip).direction;
    Results(ip).levels = levels;
    Results(ip).CABS = cell(numel(levels),1);
    Results(ip).CIBS = cell(numel(levels),1);
    Results(ip).thetaPlant = cell(numel(levels),1);
    Results(ip).epsilonVec = cell(numel(levels),1);
    Results(ip).gPlant = zeros(numel(levels),1);
    Results(ip).gCtrl = gCtrl*ones(numel(levels),1);
    Results(ip).gRatio = zeros(numel(levels),1);
    for il = 1:numel(levels)
        p = levels(il);
        epsilonVec = p*patterns(ip).direction;
        thetaPlant = P.thetaTrue.*(1+epsilonVec);
        Pcase = P;
        Pcase.thetaPlant = thetaPlant;
        fprintf('%s - maximum error %.0f%%\n',patterns(ip).name,100*p);
        fprintf('Plant parameter perturbations [%%]:\n');
        disp(100*epsilonVec.');
        %% Same physical plant for both controllers
        RCABS = simulate_controller('CABS',Pcase,C,cabsOptions);
        RCIBS = simulate_controller('CIBS',Pcase,C);
        Results(ip).CABS{il} = RCABS;
        Results(ip).CIBS{il} = RCIBS;
        Results(ip).thetaPlant{il} = thetaPlant;
        Results(ip).epsilonVec{il} = epsilonVec;
        gPlant = P.Kq*(thetaPlant(6)*P.M+thetaPlant(7));
        Results(ip).gPlant(il) = gPlant;
        Results(ip).gRatio(il) = gPlant/gCtrl;
        fprintf('\nCABS:\n');
        print_metrics(RCABS);
        fprintf('\nCIBS:\n');
        print_metrics(RCIBS);
        fprintf('\nCIBS gPlant/gCtrl = %.5f\n',Results(ip).gRatio(il));
    end
end
%% QUANTITATIVE TABLES
comparisonTable = build_complete_table(Nominal,Results);
fprintf('MATCHED CABS / CIBS UNCERTAINTY COMPARISON\n');
disp(comparisonTable);
mainIndex = find(abs(levels-mainLevel)<1e-12,1);
mainLabel = "max " + string(round(100*mainLevel)) + "%";
mainMask = comparisonTable.Case == "Nominal" | contains(comparisonTable.Case,mainLabel);
mainTable = comparisonTable(mainMask,1:9);
fprintf('MAIN MATCHED COMPARISON - MAXIMUM ERROR %.0f%%\n',100*mainLevel);
disp(mainTable);

%% 30% TIME-DOMAIN COMPARISONS
for ip = 1:numel(Results)
    RCABS = Results(ip).CABS{mainIndex};
    RCIBS = Results(ip).CIBS{mainIndex};
    plot_matched_dashboard(RCABS,RCIBS,P,Results(ip).name,mainLevel);
end
%% PERFORMANCE VS UNCERTAINTY
plot_metric_comparison(Results,levels,Nominal, ...
    'RMSE_alpha_deg','RMSE_\alpha [deg]', ...
    'CABS vs CIBS - Alpha RMSE', ...
    'Matched-Uncertainty Angle-of-Attack Performance');
plot_metric_comparison(Results,levels,Nominal, ...
    'RMSE_q_deg_s','RMSE_q [deg/s]', ...
    'CABS vs CIBS - q RMSE', ...
    'Matched-Uncertainty Pitch-Rate Performance');
plot_metric_comparison(Results,levels,Nominal, ...
    'deltaRMS_deg','\delta_{RMS} [deg]', ...
    'CABS vs CIBS - Elevator RMS', ...
    'Matched-Uncertainty Control Effort');
plot_control_effectiveness_ratio(Results,levels);
%% CABS PARAMETER ADAPTATION AT 30%
for ip = 1:numel(Results)
    RCABS = Results(ip).CABS{mainIndex};
    thetaPlant = Results(ip).thetaPlant{mainIndex};
    plot_cabs_parameter_adaptation(RCABS,P,thetaPlant, ...
        Results(ip).name,mainLevel);
end
%% SAVE RESULTS
save(fullfile(E.resultsDir,'CABS_CIBS_matched_uncertainty_comparison.mat'), ...
    'Nominal','Results','comparisonTable','mainTable', ...
    'patterns','levels','mainLevel','P','C');
fprintf('\nResults saved to:\n');
fprintf('%s\n',fullfile(E.resultsDir,'CABS_CIBS_matched_uncertainty_comparison.mat'));
fprintf('\nMatched comparison complete.\n');
%% LOCAL HELPER FUNCTIONS
function print_metrics(R)
M = R.metrics;
fprintf('RMSE alpha       = %.5f deg\n',M.RMSE_alpha_deg);
fprintf('Max alpha error  = %.5f deg\n',M.maxAbs_alpha_deg);
fprintf('RMSE q           = %.5f deg/s\n',M.RMSE_q_deg_s);
fprintf('Max q error      = %.5f deg/s\n',M.maxAbs_q_error_deg_s);
fprintf('Delta RMS        = %.5f deg\n',M.deltaRMS_deg);
fprintf('Max delta        = %.5f deg\n',M.maxAbs_delta_deg);
fprintf('Saturation       = %.3f %%\n',M.saturationPercent);
end
%% COMPLETE TABLE
function T = build_complete_table(Nominal,Results)
T = metric_row("Nominal","CABS",Nominal.CABS,1);
T = [T; metric_row("Nominal","CIBS",Nominal.CIBS,1)];
for ip = 1:numel(Results)
    for il = 1:numel(Results(ip).levels)
        p = round(100*Results(ip).levels(il));
        caseName = string(sprintf('%s - max %d%%',Results(ip).name,p));
        T = [T; metric_row(caseName,"CABS", ...
            Results(ip).CABS{il},Results(ip).gRatio(il))];
        T = [T; metric_row(caseName,"CIBS", ...
            Results(ip).CIBS{il},Results(ip).gRatio(il))];
    end
end
end
%% TABLE ROW
function T = metric_row(caseName,controllerName,R,gRatio)
M = R.metrics;
T = table( ...
    string(caseName), ...
    string(controllerName), ...
    M.RMSE_alpha_deg, ...
    M.maxAbs_alpha_deg, ...
    M.RMSE_q_deg_s, ...
    M.maxAbs_q_error_deg_s, ...
    M.deltaRMS_deg, ...
    M.maxAbs_delta_deg, ...
    M.saturationPercent, ...
    gRatio, ...
    'VariableNames',{ ...
    'Case', ...
    'Controller', ...
    'RMSE_alpha_deg', ...
    'MaxAlphaErr_deg', ...
    'RMSE_q_deg_s', ...
    'MaxQErr_deg_s', ...
    'DeltaRMS_deg', ...
    'MaxDelta_deg', ...
    'Saturation_pct', ...
    'gPlant_over_gCtrl'});
end
%% 30% MATCHED DASHBOARD
function plot_matched_dashboard(RCABS,RCIBS,P,caseName,p)
t = RCABS.t;
figure('Name',sprintf('CABS vs CIBS - %s %.0f%%',caseName,100*p),'Color','w');
tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
%% Angle-of-attack tracking
nexttile;
plot(t,rad2deg(RCABS.alphaRef),'k--','LineWidth',1.5);
hold on;
plot(t,rad2deg(RCABS.alpha),'LineWidth',1.4);
plot(t,rad2deg(RCIBS.alpha),'LineWidth',1.4);
grid on;
xlabel('Time [s]');
ylabel('\alpha [deg]');
title('Angle-of-Attack Tracking');
legend('Reference','CABS','CIBS','Location','best');
%% Angle-of-attack error
nexttile;
plot(t,rad2deg(RCABS.alpha-RCABS.alphaRef),'LineWidth',1.4);
hold on;
plot(t,rad2deg(RCIBS.alpha-RCIBS.alphaRef),'LineWidth',1.4);
grid on;
xlabel('Time [s]');
ylabel('e_\alpha [deg]');
title('Angle-of-Attack Tracking Error');
legend('CABS','CIBS','Location','best');
%% Pitch-rate tracking error
nexttile;
plot(t,rad2deg(RCABS.q-RCABS.qc),'LineWidth',1.4);
hold on;
plot(t,rad2deg(RCIBS.q-RCIBS.qc),'LineWidth',1.4);
grid on;
xlabel('Time [s]');
ylabel('e_q [deg/s]');
title('Pitch-Rate Tracking Error');
legend('CABS','CIBS','Location','best');
%% Elevator command
nexttile;
plot(t,rad2deg(RCABS.delta),'LineWidth',1.4);
hold on;
plot(t,rad2deg(RCIBS.delta),'LineWidth',1.4);
yline(rad2deg(P.deltaMax),'k--');
yline(rad2deg(P.deltaMin),'k--');
grid on;
xlabel('Time [s]');
ylabel('\delta [deg]');
title('Elevator Command');
legend('CABS','CIBS','Limit','Location','best');
title(tl,sprintf( ...
    'Matched CABS--CIBS Comparison: %s, Maximum Error %.0f%%', ...
    caseName,100*p));
end
%% GENERIC RMSE / CONTROL-EFFORT COMPARISON
function plot_metric_comparison(Results,levels,Nominal,metricName,yLabel,figName,mainTitle)
figure('Name',figName,'Color','w');
tl = tiledlayout(1,numel(Results),'TileSpacing','compact','Padding','compact');
x = [0 100*levels];
for ip = 1:numel(Results)
    cabsMetric = zeros(1,numel(levels));
    cibsMetric = zeros(1,numel(levels));
    for il = 1:numel(levels)
        cabsMetric(il) = Results(ip).CABS{il}.metrics.(metricName);
        cibsMetric(il) = Results(ip).CIBS{il}.metrics.(metricName);
    end
    cabsMetric = [Nominal.CABS.metrics.(metricName) cabsMetric];
    cibsMetric = [Nominal.CIBS.metrics.(metricName) cibsMetric];
    nexttile;
    plot(x,cabsMetric,'-o','LineWidth',1.5,'MarkerSize',7);
    hold on;
    plot(x,cibsMetric,'-o','LineWidth',1.5,'MarkerSize',7);
    grid on;
    xlabel('Maximum individual parameter error [%]');
    ylabel(yLabel);
    title(Results(ip).name);
    legend('CABS','CIBS','Location','best');
end
title(tl,mainTitle);
end
%% CIBS CONTROL-EFFECTIVENESS RATIO
function plot_control_effectiveness_ratio(Results,levels)
figure('Name','CIBS Control Effectiveness Ratio','Color','w');
hold on;
for ip = 1:numel(Results)
    plot([0 100*levels],[1;Results(ip).gRatio(:)], ...
        '-o','LineWidth',1.5,'MarkerSize',7);
end
grid on;
xlabel('Maximum individual parameter error [%]');
ylabel('$g_{q,\mathrm{plant}}/g_{q,\mathrm{ctrl}}$','Interpreter','latex');
title('Actual-to-Assumed CIBS Pitch Control Effectiveness');
legend({Results.name},'Location','best');
end
%% CABS PARAMETER EVOLUTION
function plot_cabs_parameter_adaptation(R,P,thetaPlant,caseName,p)
figure('Name',sprintf('CABS adaptation - %s %.0f%%',caseName,100*p),'Color','w');
tl = tiledlayout(4,2,'TileSpacing','compact','Padding','compact');
for i = 1:7
    nexttile;
    plot(R.t,R.thetaHat(:,i),'LineWidth',1.2);
    hold on;
    yline(P.thetaTrue(i),'k--');
    yline(thetaPlant(i),'k:');
    grid on;
    xlabel('Time [s]');
    ylabel(sprintf('\\theta_%d',i));
    title(sprintf('Parameter Estimate \\theta_%d',i));
    if i == 1
        legend('Estimate','Nominal','Actual plant','Location','best');
    end
end
nexttile;
plot(R.t,rad2deg(R.q-R.qc),'LineWidth',1.2);
grid on;
xlabel('Time [s]');
ylabel('e_q [deg/s]');
title('CABS Inner-Loop Error');
title(tl,sprintf( ...
    'CABS Adaptation under Matched Plant Uncertainty: %s, %.0f%%', ...
    caseName,100*p));
end