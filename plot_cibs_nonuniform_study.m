function plot_cibs_nonuniform_study(D,P,C,resultsDir)
%plots for the deterministic nonuniform CIBS robustness study.
if ~exist(resultsDir,'dir')
    mkdir(resultsDir);
end %%see run_project.m - line 18
levelsPct = 100*D.levels;
families = {'drift','control','full'};
familyLabels = { ...
    'Drift uncertainty only', ...
    'Control-effectiveness uncertainty only', ...
    'Full nonuniform uncertainty'};
%% COLLECT METRICS
nLevels = numel(D.levels);
alphaRMSE = zeros(nLevels,3);
qRMSE = zeros(nLevels,3);
deltaRMS = zeros(nLevels,3);
gRatio = zeros(nLevels,3);
for f = 1:3
    family = families{f};
    for i = 1:nLevels
        R = D.(family){i};
        alphaRMSE(i,f) = R.metrics.RMSE_alpha_deg;
        qRMSE(i,f) = R.metrics.RMSE_q_deg_s;
        deltaRMS(i,f) = R.metrics.deltaRMS_deg;
        gRatio(i,f) = R.gRatio;
    end
end
nomAlphaRMSE = D.nominal.metrics.RMSE_alpha_deg;
nomQRMSE = D.nominal.metrics.RMSE_q_deg_s;
%% FIGURE 1 - NONUNIFORM PARAMETER PATTERNS AT 30%
[~,idx30] = min(abs(D.levels-0.30));
level30 = D.levels(idx30);
patternMatrix = 100*level30*[D.patterns.drift,D.patterns.control,D.patterns.full];
fig1 = figure('Name','CIBS Nonuniform Parameter Patterns');
bar(1:7,patternMatrix,'grouped');
grid on;
xlabel('Parameter index');
ylabel('Parameter perturbation [%]');
title(sprintf('CIBS Nonuniform Parameter Perturbations - Maximum Level %.0f%%',100*level30));
xticks(1:7);
xticklabels({'\theta_1','\theta_2','\theta_3','\theta_4','\theta_5','\theta_6','\theta_7'});
legend(familyLabels,'Location','best');
exportgraphics(fig1,fullfile(resultsDir,'cibs_nonuniform_parameter_patterns_30pct.png'),'Resolution',300);
%% FIGURE 2 - ALPHA RMSE
fig2 = figure('Name','CIBS Nonuniform Alpha RMSE');
hold on;
for f = 1:3
    plot(levelsPct,alphaRMSE(:,f),'-o','LineWidth',1.5,'MarkerSize',7);
end
yline(nomAlphaRMSE,'--','Nominal');
grid on;
xlabel('Maximum individual parameter error [%]');
ylabel('$\mathrm{RMSE}_{\alpha}$ [deg]','Interpreter','latex');
title('CIBS Tracking Performance under Nonuniform Parameter Uncertainty');
legend(familyLabels,'Location','best');
hold off;
exportgraphics(fig2,fullfile(resultsDir,'cibs_nonuniform_alpha_rmse.png'),'Resolution',300);
%% FIGURE 3 - Q RMSE
fig3 = figure('Name','CIBS Nonuniform q RMSE');
hold on;
for f = 1:3
    plot(levelsPct,qRMSE(:,f),'-o','LineWidth',1.5,'MarkerSize',7);
end
yline(nomQRMSE,'--','Nominal');
grid on;
xlabel('Maximum individual parameter error [%]');
ylabel('$\mathrm{RMSE}_{q}$ [deg/s]','Interpreter','latex');
title('CIBS Pitch-Rate Performance under Nonuniform Parameter Uncertainty');
legend(familyLabels,'Location','best');
hold off;
exportgraphics(fig3,fullfile(resultsDir,'cibs_nonuniform_q_rmse.png'),'Resolution',300);
%% FIGURE 4 - ACTUAL / ASSUMED CONTROL EFFECTIVENESS
fig4 = figure('Name','CIBS Nonuniform Control Effectiveness');
hold on;
for f = 1:3
    plot(levelsPct,gRatio(:,f),'-o','LineWidth',1.5,'MarkerSize',7);
end
yline(1,'--','Nominal');
grid on;
xlabel('Maximum individual parameter error [%]');
ylabel('$g_{q,\mathrm{plant}}/g_{q,\mathrm{ctrl}}$','Interpreter','latex');
title('Actual-to-Assumed Pitch Control Effectiveness');
legend(familyLabels,'Location','best');
hold off;
exportgraphics(fig4,fullfile(resultsDir,'cibs_nonuniform_control_effectiveness.png'),'Resolution',300);
%% FIGURE 5 - 30% TIME-DOMAIN DASHBOARD
Rn = D.nominal;
Rd = D.drift{idx30};
Rg = D.control{idx30};
Rf = D.full{idx30};
fig5 = figure('Name','CIBS Nonuniform 30 Percent Dashboard');
tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
%% Alpha tracking
nexttile;
plot(Rn.t,rad2deg(Rn.alphaRef),'--','LineWidth',1.4);
hold on;
plot(Rn.t,rad2deg(Rn.alpha),'LineWidth',1.2);
plot(Rd.t,rad2deg(Rd.alpha),'LineWidth',1.2);
plot(Rg.t,rad2deg(Rg.alpha),'LineWidth',1.2);
plot(Rf.t,rad2deg(Rf.alpha),'LineWidth',1.2);
grid on;
xlabel('Time [s]');
ylabel('\alpha [deg]');
title('Angle-of-Attack Tracking');
legend('Reference','Nominal','Drift only','Control effectiveness only','Full nonuniform','Location','best');
hold off;
%% Alpha tracking error
nexttile;
plot(Rn.t,rad2deg(Rn.alpha-Rn.alphaRef),'LineWidth',1.2);
hold on;
plot(Rd.t,rad2deg(Rd.alpha-Rd.alphaRef),'LineWidth',1.2);
plot(Rg.t,rad2deg(Rg.alpha-Rg.alphaRef),'LineWidth',1.2);
plot(Rf.t,rad2deg(Rf.alpha-Rf.alphaRef),'LineWidth',1.2);
grid on;
xlabel('Time [s]');
ylabel('e_\alpha [deg]');
title('Angle-of-Attack Tracking Error');
legend('Nominal','Drift only','Control effectiveness only','Full nonuniform','Location','best');
hold off;
%% Pitch-rate tracking error
nexttile;
plot(Rn.t,rad2deg(Rn.q-Rn.qc),'LineWidth',1.2);
hold on;
plot(Rd.t,rad2deg(Rd.q-Rd.qc),'LineWidth',1.2);
plot(Rg.t,rad2deg(Rg.q-Rg.qc),'LineWidth',1.2);
plot(Rf.t,rad2deg(Rf.q-Rf.qc),'LineWidth',1.2);
grid on;
xlabel('Time [s]');
ylabel('e_q [deg/s]');
title('Pitch-Rate Tracking Error');
legend('Nominal','Drift only','Control effectiveness only','Full nonuniform','Location','best');
hold off;
%% Elevator
nexttile;
plot(Rn.t,rad2deg(Rn.delta),'LineWidth',1.2);
hold on;
plot(Rd.t,rad2deg(Rd.delta),'LineWidth',1.2);
plot(Rg.t,rad2deg(Rg.delta),'LineWidth',1.2);
plot(Rf.t,rad2deg(Rf.delta),'LineWidth',1.2);
yline(rad2deg(P.deltaMax),'--');
yline(rad2deg(P.deltaMin),'--');
grid on;
xlabel('Time [s]');
ylabel('\delta [deg]');
title('Elevator Command');
legend('Nominal','Drift only','Control effectiveness only','Full nonuniform','Location','best');
hold off;
title(tl,sprintf('CIBS Nonuniform Robustness Comparison - Maximum Error %.0f%%',100*level30));
exportgraphics(fig5,fullfile(resultsDir,'cibs_nonuniform_30pct_dashboard.png'),'Resolution',300);
end