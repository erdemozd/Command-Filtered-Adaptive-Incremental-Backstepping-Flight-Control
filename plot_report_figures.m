function plot_report_figures(CABS,CIBS,P,C,E)
%% 1) CABS ADAPTATION EFFECT
fig = figure('Name','Report - CABS adaptation effect','WindowState','maximized');
plot(CABS.wrongOff.t,rad2deg(CABS.wrongOff.alpha-CABS.wrongOff.alphaRef),'LineWidth',1.2);
hold on;
plot(CABS.wrongOn.t,rad2deg(CABS.wrongOn.alpha-CABS.wrongOn.alphaRef),'LineWidth',1.2);
grid on;
xlabel('Time [s]');
ylabel('e_\alpha [deg]');
legend('Adaptation OFF','Adaptation ON','Location','best');
title('CABS Angle-of-Attack Error: Adaptation Effect');
save_plot(fig,E,'report_cabs_adaptation_error');
%% 2) CABS NORMALIZED PARAMETER EVOLUTION
fig = figure('Name','Report - CABS parameter evolution','WindowState','maximized');
plot(CABS.wrongOn.t,CABS.wrongOn.rhoHat,'LineWidth',1.1);
hold on;
yline(1,'--','LineWidth',1.1);
grid on;
xlabel('Time [s]');
ylabel('$\hat{\rho}_i$','Interpreter','latex');
title('CABS Normalized Parameter Estimate Evolution');
legend( ...
    '$\hat{\rho}_1$','$\hat{\rho}_2$','$\hat{\rho}_3$', ...
    '$\hat{\rho}_4$','$\hat{\rho}_5$','$\hat{\rho}_6$', ...
    '$\hat{\rho}_7$','True normalized value', ...
    'Interpreter','latex','Location','bestoutside');
save_plot(fig,E,'report_cabs_normalized_parameter_evolution');
%% 3) COMMAND FILTER RESPONSE
Rfilter = CABS.perfect;
fig = figure('Name','Report - Command Filter Response','WindowState','maximized');
tl = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
%% Raw vs filtered virtual rate command
nexttile;
plot(Rfilter.t,rad2deg(Rfilter.qRaw),'--','LineWidth',1.3);
hold on;
plot(Rfilter.t,rad2deg(Rfilter.qc),'LineWidth',1.3);
grid on;
xlabel('Time [s]');
ylabel('Pitch Rate [deg/s]');
legend('$q_c^0$','$q_c$','Interpreter','latex','Location','best');
title('Raw and Filtered Virtual Pitch-Rate Commands');
%% Command-filter generated derivative
nexttile;
plot(Rfilter.t,rad2deg(Rfilter.qcDot),'LineWidth',1.3);
grid on;
xlabel('Time [s]');
ylabel('$\dot{q}_c$ [deg/s$^2$]','Interpreter','latex');
title('Command-Filter Generated Derivative');
title(tl,sprintf('Command Filter Response: \\omega_n = %.0f rad/s, \\zeta = %.1f',C.wn,C.zeta));
save_plot(fig,E,'report_command_filter_response');
%% COMMON CIBS ROBUSTNESS ARRAYS
u = [-fliplr(CIBS.levels),0,CIBS.levels]*100;
RmseA = zeros(size(u));
RmseQ = zeros(size(u));
maxA = zeros(size(u));
dRMS = zeros(size(u));
diRMS = zeros(size(u));
n = numel(CIBS.levels);
%% Negative uncertainty cases
for i = 1:n
    index = n-i+1;
    RmseA(i) = CIBS.minus{index}.metrics.RMSE_alpha_deg;
    RmseQ(i) = CIBS.minus{index}.metrics.RMSE_q_deg_s;
    maxA(i) = CIBS.minus{index}.metrics.maxAbs_alpha_deg;
    dRMS(i) = CIBS.minus{index}.metrics.deltaRMS_deg;
    diRMS(i) = CIBS.minus{index}.metrics.deltaIncRMS_deg;
end
%% Nominal case
RmseA(n+1) = CIBS.nominal.metrics.RMSE_alpha_deg;
RmseQ(n+1) = CIBS.nominal.metrics.RMSE_q_deg_s;
maxA(n+1) = CIBS.nominal.metrics.maxAbs_alpha_deg;
dRMS(n+1) = CIBS.nominal.metrics.deltaRMS_deg;
diRMS(n+1) = CIBS.nominal.metrics.deltaIncRMS_deg;
%% Positive uncertainty cases
for i = 1:n
    j = n+1+i;
    RmseA(j) = CIBS.plus{i}.metrics.RMSE_alpha_deg;
    RmseQ(j) = CIBS.plus{i}.metrics.RMSE_q_deg_s;
    maxA(j) = CIBS.plus{i}.metrics.maxAbs_alpha_deg;
    dRMS(j) = CIBS.plus{i}.metrics.deltaRMS_deg;
    diRMS(j) = CIBS.plus{i}.metrics.deltaIncRMS_deg;
end
%% 4) CIBS ANGLE-OF-ATTACK RMSE
fig = figure('Name','Report - CIBS alpha RMSE','WindowState','maximized');
plot(u,RmseA,'-o','LineWidth',1.5);
hold on;
xline(0,'--');
grid on;
xlabel('Plant Parameter Error [%]');
ylabel('Angle-of-Attack RMSE [deg]');
title('CIBS Robustness: Tracking RMSE vs Plant Parameter Error');
save_plot(fig,E,'report_cibs_alpha_rmse_vs_uncertainty');
%% 5) CIBS PITCH-RATE RMSE
fig = figure('Name','Report - CIBS q RMSE','WindowState','maximized');
plot(u,RmseQ,'-o','LineWidth',1.5);
hold on;
xline(0,'--');
grid on;
xlabel('Plant Parameter Error [%]');
ylabel('Pitch-Rate RMSE [deg/s]');
title('CIBS Robustness: Pitch-Rate RMSE vs Plant Parameter Error');
save_plot(fig,E,'report_cibs_q_rmse_vs_uncertainty');
%% 6) ACTUAL VS ASSUMED CONTROL EFFECTIVENESS
gCtrl = P.Kq*(P.thetaTrue(6)*P.M+P.thetaTrue(7));
gPlant = gCtrl.*(1+u/100);
fig = figure('Name','Report - CIBS control effectiveness','WindowState','maximized');
plot(u,gPlant,'-o','LineWidth',1.5);
hold on;
yline(gCtrl,'--','LineWidth',1.2);
grid on;
xlabel('Plant Parameter Error [%]');
ylabel('Pitch Control Effectiveness');
legend('Actual plant $g_q$','Controller-assumed $g_q$', ...
    'Interpreter','latex','Location','best');
title('CIBS: Actual vs Assumed Control Effectiveness');
save_plot(fig,E,'report_cibs_actual_vs_assumed_control_effectiveness');
%% 7) RELATIVE CIBS ANGLE-OF-ATTACK RMSE CHANGE
nominalRMSE = CIBS.nominal.metrics.RMSE_alpha_deg;
rel = 100*(RmseA-nominalRMSE)/nominalRMSE;
fig = figure('Name','Report - CIBS relative degradation','WindowState','maximized');
plot(u,rel,'-o','LineWidth',1.5);
hold on;
xline(0,'--');
yline(0,'--');
grid on;
xlabel('Plant Parameter Error [%]');
ylabel('Relative change in \alpha RMSE [%]');
title('CIBS Relative Tracking Degradation');
save_plot(fig,E,'report_cibs_relative_tracking_degradation');
%% 8) CROSS-CONTROLLER PERFORMANCE VS CONTROL EFFORT
labels = { ...
    'CABS wrong/off', ...
    'CABS wrong/on', ...
    'CIBS nominal', ...
    'CIBS -60%', ...
    'CIBS +60%'};

Rs = { ...
    CABS.wrongOff, ...
    CABS.wrongOn, ...
    CIBS.nominal, ...
    CIBS.minus{end}, ...
    CIBS.plus{end}};

x = zeros(1,numel(Rs));
y = zeros(1,numel(Rs));

for i = 1:numel(Rs)
    x(i) = Rs{i}.metrics.deltaRMS_deg;
    y(i) = Rs{i}.metrics.RMSE_alpha_deg;
end

fig = figure('Name','Report - performance vs effort','WindowState','maximized');
scatter(x,y,70,'filled');
grid on;
xlabel('Elevator RMS [deg]');
ylabel('Angle-of-Attack RMSE [deg]');
title('Tracking Performance vs Control Effort');
for i = 1:numel(Rs)
    text(x(i),y(i),['  ' labels{i}],'VerticalAlignment','bottom');
end
save_plot(fig,E,'report_tracking_performance_vs_control_effort');
end
%% LOCAL FIGURE-SAVING HELPER / not necessarily needed, just as i said in the run_project.m, I just like to save my figures automatically by running scripts rather than saving them manually
function save_plot(fig,E,baseName)
%SAVE_PLOT Saves a figure according to the project export settings.
if ~exist(E.figuresDir,'dir')
    mkdir(E.figuresDir);
end
baseName = regexprep(lower(baseName),'[^a-z0-9_\-]','_');
if E.savePNG
    exportgraphics(fig,fullfile(E.figuresDir,[baseName '.png']),'Resolution',E.figureResolution);
end
if E.savePDF
    exportgraphics(fig,fullfile(E.figuresDir,[baseName '.pdf']),'ContentType','vector');
end
if E.saveFIG
    savefig(fig,fullfile(E.figuresDir,[baseName '.fig']));
end
fprintf('Saved figure: %s\n',fullfile(E.figuresDir,baseName));
end