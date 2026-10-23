function plot_dashboard(mode,data,P,E)
%All technical dashboards in one plotting file.
E = ensure_plot_settings(E);
switch lower(mode)
    case 'single'
        single_dashboard(data,P,E);
    case 'cabs_compare'
        cabs_compare(data,P,E);
    case 'cibs_robustness'
        cibs_robustness(data,P,E);
end
end
%% SINGLE NOMINAL DASHBOARD
function single_dashboard(R,P,E)
fig = figure('Name',[R.controllerType ' nominal'],'NumberTitle','off','WindowState','maximized');
tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
title(tl,[R.controllerType ' - Nominal Performance']);
core4(R,P);
save_plot(fig,E,sprintf('%s_nominal_dashboard',lower(R.controllerType)));
end
%% CABS COMPARISON DASHBOARDS
function cabs_compare(D,P,E)
cases = {D.perfect,D.wrongOff,D.wrongOn};
titles = { ...
    'CABS CASE 1 - Perfect Model', ...
    'CABS CASE 2 - Wrong Model, Adaptation OFF', ...
    'CABS CASE 3 - Wrong Model, Adaptation ON'};

names = { ...
    'cabs_case1_perfect_model', ...
    'cabs_case2_wrong_model_adaptation_off', ...
    'cabs_case3_wrong_model_adaptation_on'};
for c = 1:3
    R = cases{c};
    fig = figure('Name',titles{c},'NumberTitle','off','WindowState','maximized');
    tl = tiledlayout(3,4,'TileSpacing','compact','Padding','compact');
    title(tl,titles{c});
    %% Tracking
    nexttile;
    tracking(R);
    %% Alpha error
    nexttile;
    alpha_error(R);
    %% Inner rate loop
    nexttile;
    rate_loop(R);
    %% Elevator
    nexttile;
    elevator(R,P);
    %% Normalized parameter estimates
    nexttile;
    plot(R.t,R.rhoHat,'LineWidth',1);
    hold on;
    yline(1,'--');
    grid on;
    xlabel('Time [s]');
    ylabel('\hat{\rho}_i');
    title('Normalized Parameter Estimates');
    %% Individual physical parameter estimates
    for i = 1:7
        nexttile;
        plot(R.t,R.thetaHat(:,i),'LineWidth',1.1);
        hold on;
        yline(P.thetaTrue(i),'--');
        grid on;
        xlabel('Time [s]');
        ylabel(sprintf('\\theta_%d',i));
        title(sprintf('Parameter Estimate \\theta_%d',i));
    end
    save_plot(fig,E,names{c});
end
end
%% CIBS UNIFORM ROBUSTNESS DASHBOARDS
function cibs_robustness(D,P,E)
for i = 1:numel(D.levels)
    p = round(100*D.levels(i));
    Rm = D.minus{i};
    Rp = D.plus{i};
    Rn = D.nominal;
    ttl = sprintf('CIBS ROBUSTNESS - +/- %d%% Plant Parameter Error',p);
    fig = figure('Name',ttl,'NumberTitle','off','WindowState','maximized');
    tl = tiledlayout(2,4,'TileSpacing','compact','Padding','compact');
    title(tl,ttl);
    %% Angle-of-attack tracking
    nexttile;
    plot(Rn.t,rad2deg(Rn.alphaRef),'--','LineWidth',1.2);
    hold on;
    plot(Rn.t,rad2deg(Rn.alpha),'LineWidth',1.1);
    plot(Rm.t,rad2deg(Rm.alpha),'LineWidth',1.1);
    plot(Rp.t,rad2deg(Rp.alpha),'LineWidth',1.1);
    grid on;
    xlabel('Time [s]');
    ylabel('\alpha [deg]');
    title('Angle-of-Attack Tracking');
    legend('\alpha_r','Nominal',sprintf('-%d%%',p),sprintf('+%d%%',p),'Location','best');
    %% Alpha tracking error
    nexttile;
    plot(Rn.t,rad2deg(Rn.alpha-Rn.alphaRef),'LineWidth',1.1);
    hold on;
    plot(Rm.t,rad2deg(Rm.alpha-Rm.alphaRef),'LineWidth',1.1);
    plot(Rp.t,rad2deg(Rp.alpha-Rp.alphaRef),'LineWidth',1.1);
    grid on;
    xlabel('Time [s]');
    ylabel('e_\alpha [deg]');
    title('Angle-of-Attack Error');
    legend('Nominal',sprintf('-%d%%',p),sprintf('+%d%%',p),'Location','best');
    %% Pitch-rate response
    nexttile;
    plot(Rn.t,rad2deg(Rn.qc),'--','LineWidth',1.2);
    hold on;
    plot(Rn.t,rad2deg(Rn.q),'LineWidth',1.1);
    plot(Rm.t,rad2deg(Rm.q),'LineWidth',1.1);
    plot(Rp.t,rad2deg(Rp.q),'LineWidth',1.1);
    grid on;
    xlabel('Time [s]');
    ylabel('q [deg/s]');
    title('Inner Pitch-Rate Response');
    legend('q_c nominal','Nominal',sprintf('-%d%%',p),sprintf('+%d%%',p),'Location','best');
    %% Elevator
    nexttile;
    plot(Rn.t,rad2deg(Rn.delta),'LineWidth',1.1);
    hold on;
    plot(Rm.t,rad2deg(Rm.delta),'LineWidth',1.1);
    plot(Rp.t,rad2deg(Rp.delta),'LineWidth',1.1);
    yline(rad2deg(P.deltaMax),'--');
    yline(rad2deg(P.deltaMin),'--');
    grid on;
    xlabel('Time [s]');
    ylabel('\delta [deg]');
    title('Elevator Command');
    legend('Nominal',sprintf('-%d%%',p),sprintf('+%d%%',p),'limits','Location','best');
    %% Pitch-rate tracking error
    nexttile;
    plot(Rn.t,rad2deg(Rn.q-Rn.qc),'LineWidth',1.1);
    hold on;
    plot(Rm.t,rad2deg(Rm.q-Rm.qc),'LineWidth',1.1);
    plot(Rp.t,rad2deg(Rp.q-Rp.qc),'LineWidth',1.1);
    grid on;
    xlabel('Time [s]');
    ylabel('q-q_c [deg/s]');
    title('Inner Rate Tracking Error');
    legend('Nominal',sprintf('-%d%%',p),sprintf('+%d%%',p),'Location','best');
    %% Incremental elevator
    nexttile;
    plot(Rn.t,rad2deg(Rn.deltaInc),'LineWidth',1.1);
    hold on;
    plot(Rm.t,rad2deg(Rm.deltaInc),'LineWidth',1.1);
    plot(Rp.t,rad2deg(Rp.deltaInc),'LineWidth',1.1);
    grid on;
    xlabel('Time [s]');
    ylabel('\Delta\delta [deg]');
    title('Incremental Elevator Command');
    legend('Nominal',sprintf('-%d%%',p),sprintf('+%d%%',p),'Location','best');
    %% Incremental acceleration error
    nexttile;
    tn = Rn.t(1:end-1);
    plot(tn,rad2deg(Rn.qDot(2:end)-Rn.nu(1:end-1)),'LineWidth',1.1);
    hold on;
    plot(tn,rad2deg(Rm.qDot(2:end)-Rm.nu(1:end-1)),'LineWidth',1.1);
    plot(tn,rad2deg(Rp.qDot(2:end)-Rp.nu(1:end-1)),'LineWidth',1.1);
    grid on;
    xlabel('Time [s]');
    ylabel('\dot{q}(k+1)-\nu_q(k) [deg/s^2]');
    title('Incremental Acceleration Error');
    legend('Nominal',sprintf('-%d%%',p),sprintf('+%d%%',p),'Location','best');
    %% Actual vs assumed control effectiveness
    nexttile;
    gCtrl = Rn.gInner(1);
    gMinus = (1-D.levels(i))*gCtrl;
    gPlus = (1+D.levels(i))*gCtrl;
    yline(gCtrl,'-','Controller');
    hold on;
    yline(gMinus,'--',sprintf('Plant -%d%%',p));
    yline(gPlus,':',sprintf('Plant +%d%%',p));
    grid on;
    ylabel('Pitch control effectiveness');
    title('Actual vs Assumed g_q');
    save_plot(fig,E,sprintf('cibs_robustness_plus_minus_%02d_percent',p));
end
end
%% COMMON FOUR PANEL DASHBOARD
function core4(R,P)
nexttile;
tracking(R);
nexttile;
alpha_error(R);
nexttile;
rate_loop(R);
nexttile;
elevator(R,P);
end
%% TRACKING
function tracking(R)
plot(R.t,rad2deg(R.alphaRef),'--','LineWidth',1.2);
hold on;
plot(R.t,rad2deg(R.alpha),'LineWidth',1.2);
grid on;
xlabel('Time [s]');
ylabel('\alpha [deg]');
legend('\alpha_r','\alpha','Location','best');
title('Angle-of-Attack Tracking');
end

%% ALPHA ERROR
function alpha_error(R)
plot(R.t,rad2deg(R.alpha-R.alphaRef),'LineWidth',1.2);
grid on;
xlabel('Time [s]');
ylabel('Error [deg]');
title('Angle-of-Attack Error');
end
%% RATE LOOP
function rate_loop(R)
plot(R.t,rad2deg(R.qc),'--','LineWidth',1.2);
hold on;
plot(R.t,rad2deg(R.q),'LineWidth',1.2);
grid on;
xlabel('Time [s]');
ylabel('q [deg/s]');
legend('q_c','q','Location','best');
title('Inner Rate Loop');
end
%% ELEVATOR
function elevator(R,P)
plot(R.t,rad2deg(R.deltaRaw),'--','LineWidth',1);
hold on;
plot(R.t,rad2deg(R.delta),'LineWidth',1.2);
yline(rad2deg(P.deltaMax),'--');
yline(rad2deg(P.deltaMin),'--');
grid on;
xlabel('Time [s]');
ylabel('\delta [deg]');
legend('\delta_{raw}','\delta','limits','Location','best');
title('Elevator Command');
end
%% ENSURE OUTPUT SETTINGS EXIST
function E = ensure_plot_settings(E)
if ~isfield(E,'resultsDir')
    projectRoot = fileparts(mfilename('fullpath'));
    E.resultsDir = fullfile(projectRoot,'results');
end
if ~isfield(E,'figuresDir')
    E.figuresDir = fullfile(E.resultsDir,'figures');
end
if ~isfield(E,'savePNG')
    E.savePNG = true;
end
if ~isfield(E,'savePDF')
    E.savePDF = true;
end
if ~isfield(E,'saveFIG')
    E.saveFIG = true;
end
if ~isfield(E,'figureResolution')
    E.figureResolution = 300;
end
if ~exist(E.resultsDir,'dir')
    mkdir(E.resultsDir);
end
if ~exist(E.figuresDir,'dir')
    mkdir(E.figuresDir);
end
end
%% save figure / same deal - see run_project.m - line 18
function save_plot(fig,E,baseName)
E = ensure_plot_settings(E);
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