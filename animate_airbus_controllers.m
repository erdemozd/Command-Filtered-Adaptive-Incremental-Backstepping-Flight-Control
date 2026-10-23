function D = animate_airbus_controllers(options)
%ANIMATE_AIRBUS_CONTROLLERS Simulate CABS/CIBS and export a 3-D comparison GIF.
%   D = animate_airbus_controllers() runs the full 60 s experiment.
%   D = animate_airbus_controllers(Visible='off',Fps=12,Speed=6)
%   uses the same physics with faster playback. No external mesh/toolbox needed.
%   Original generic airliner geometry; no third-party aircraft assets.
%   Pitch = integral(q dt); flight-path angle = pitch - alpha. Translation
%   is omitted because the source plant has no position/altitude states.
arguments
    options.Visible (1,:) char {mustBeMember(options.Visible,{'on','off'})} = 'on'
    options.Fps (1,1) double {mustBePositive,mustBeFinite} = 12
    options.Speed (1,1) double {mustBePositive,mustBeFinite} = 6
    options.Duration (1,1) double {mustBePositive,mustBeFinite} = 60
    options.Uncertainty (1,1) double {mustBeGreaterThanOrEqual(options.Uncertainty,0),mustBeLessThanOrEqual(options.Uncertainty,0.3)} = 0.30
    options.OutputDir (1,:) char = fullfile(fileparts(mfilename('fullpath')),'docs','media')
end
[P,C,E] = setup_project();
P.tEnd = options.Duration;
% Same full-nonuniform perturbed physical plant for both controllers.
P.thetaPlant = P.thetaTrue.*(1 + options.Uncertainty*E.CIBS.nonuniformPatterns.full);
opt.rho0 = C.CABS.rhoPerfect;
opt.adaptationOn = true;
D.CABS = simulate_controller('CABS',P,C,opt);
D.CIBS = simulate_controller('CIBS',P,C);
D.P = P; D.C = C; D.options = options;
names = {'CABS','CIBS'};
for j = 1:2
    R = D.(names{j});
    assert(all(isfinite([R.alpha;R.q;R.delta])), 'Simulation produced nonfinite states.');
    assert(all(abs(R.delta)<=P.deltaMax+1e-12), 'Elevator limit exceeded.');
    D.(names{j}).pitch = cumtrapz(R.t,R.q);
end
if ~isfolder(options.OutputDir), mkdir(options.OutputDir); end
dataDir = fullfile(fileparts(mfilename('fullpath')),'results','animation');
if ~isfolder(dataDir), mkdir(dataDir); end
Controller = string(names(:));
AlphaRMSE_deg = [D.CABS.metrics.RMSE_alpha_deg; D.CIBS.metrics.RMSE_alpha_deg];
PitchRateRMSE_deg_s = [D.CABS.metrics.RMSE_q_deg_s; D.CIBS.metrics.RMSE_q_deg_s];
ElevatorRMS_deg = [D.CABS.metrics.deltaRMS_deg; D.CIBS.metrics.deltaRMS_deg];
Saturation_pct = [D.CABS.metrics.saturationPercent; D.CIBS.metrics.saturationPercent];
D.metrics = table(Controller,AlphaRMSE_deg,PitchRateRMSE_deg_s,ElevatorRMS_deg,Saturation_pct);
save(fullfile(dataDir,'controller_animation.mat'),'D');
writetable(D.metrics,fullfile(options.OutputDir,'controller_metrics.csv'));

colors = [0.08 0.43 0.78; 0.92 0.35 0.12];
f = figure('Color','w','Position',[50 50 1200 760], ...
    'Visible',options.Visible,'Name','CABS / CIBS | 3D airliner response');
cleanup = onCleanup(@() close(f));
tl = tiledlayout(f,2,6,'TileSpacing','compact','Padding','compact');
title(tl,sprintf('CABS / CIBS  |  Matched plant uncertainty: %.0f%%',100*options.Uncertainty),'FontSize',17);
subtitle(tl,sprintf('Original airliner visual | Original Mach 2.2 plant | Pitch from integral of q | %.0fx playback',options.Speed));
for j = 1:2
    ax = nexttile(tl,(j-1)*3+1,[1 3]);
    hold(ax,'on'); axis(ax,'equal');
    xlim(ax,[-26 26]); ylim(ax,[-24 24]); zlim(ax,[-15 15]);
    view(ax,32,23); grid(ax,'on');
    xlabel(ax,'Body-forward display axis'); ylabel(ax,'Lateral'); zlabel(ax,'Up');
    title(ax,sprintf('%s  |  alpha RMSE %.4f deg',names{j},AlphaRMSE_deg(j)));
    body(j) = hgtransform('Parent',ax); %#ok<AGROW>
    elevator(j) = airliner_mesh(body(j),colors(j,:)); %#ok<AGROW>
    % Velocity direction stays distinct from body pitch (alpha is their angle).
    velocity(j) = quiver3(ax,0,0,0,22,0,0,0,'Color',[0.18 0.65 0.35], ...
        'LineWidth',2,'MaxHeadSize',0.3); %#ok<AGROW>
    hud(j) = text(ax,0.02,0.97,'','Units','normalized','VerticalAlignment','top', ...
        'FontSize',10,'FontName','Consolas'); %#ok<AGROW>
    camlight(ax,'headlight'); lighting(ax,'gouraud');
end
t = D.CABS.t;
signals = {'alpha','error','delta'};
labels = {'Angle of attack [deg]','Tracking error [deg]','Elevator [deg]'};
plotTitles = {'Angle-of-Attack Tracking','Angle-of-Attack Tracking Error','Elevator Deflection'};
for s = 1:3
    ax = nexttile(tl,6+(s-1)*2+1,[1 2]); hold(ax,'on'); grid(ax,'on');
    if s==1
        plot(ax,t,rad2deg(D.CABS.alphaRef),'k--','LineWidth',1.2,'DisplayName','Reference');
    elseif s==3
        yline(ax,rad2deg(P.deltaMax),'k:','HandleVisibility','off');
        yline(ax,rad2deg(P.deltaMin),'k:','HandleVisibility','off');
    end
    for j = 1:2
        R = D.(names{j});
        if strcmp(signals{s},'error'), y = R.alpha-R.alphaRef; else, y = R.(signals{s}); end
        y = rad2deg(y);
        plot(ax,t,y,'Color',[colors(j,:) 0.25],'HandleVisibility','off');
        traces(s,j) = plot(ax,nan,nan,'Color',colors(j,:),'LineWidth',1.6,'DisplayName',names{j}); %#ok<AGROW>
        plotData{s,j} = y; %#ok<AGROW>
    end
    xlim(ax,[0 P.tEnd]); xlabel(ax,'Simulation time [s]'); ylabel(ax,labels{s});
    title(ax,plotTitles{s},'FontSize',12,'FontWeight','bold');
    legend(ax,'Location','southoutside','Orientation','horizontal');
    cursor(s) = xline(ax,0,'Color',[0.4 0.4 0.4],'HandleVisibility','off'); %#ok<AGROW>
end
gifPath = fullfile(options.OutputDir,'controller_comparison.gif');
frameCount = max(2,round(P.tEnd/options.Speed*options.Fps)+1);
indices = unique(round(linspace(1,numel(t),frameCount)));
for n = 1:numel(indices)
    k = indices(n);
    for j = 1:2
        R = D.(names{j}); theta = R.pitch(k); gamma = theta-R.alpha(k);
        body(j).Matrix = makehgtform('yrotate',-theta);
        elevator(j).Matrix = makehgtform('translate',[-13 0 0.5])* ...
            makehgtform('yrotate',-R.delta(k));
        set(velocity(j),'UData',22*cos(gamma),'WData',22*sin(gamma));
        hud(j).String = sprintf('t %5.1f s   pitch %+6.1f deg\nalpha %+5.1f deg   q %+6.1f deg/s\nelevator %+5.1f deg', ...
            t(k),rad2deg(theta),rad2deg(R.alpha(k)),rad2deg(R.q(k)),rad2deg(R.delta(k)));
        for s = 1:3
            set(traces(s,j),'XData',t(1:k),'YData',plotData{s,j}(1:k));
            cursor(s).Value = t(k);
        end
    end
    drawnow;
    frame = getframe(f);
    [indexed,map] = rgb2ind(frame.cdata,256);
    if n==1
        imwrite(indexed,map,gifPath,'gif','LoopCount',Inf,'DelayTime',1/options.Fps);
    else
        imwrite(indexed,map,gifPath,'gif','WriteMode','append','DelayTime',1/options.Fps);
    end
    if n==round(numel(indices)/2)
        imwrite(frame.cdata,fullfile(options.OutputDir,'controller_comparison.png'));
    end
    if mod(n,20)==0, fprintf('Rendered %d/%d frames\n',n,numel(indices)); end
end
info = imfinfo(gifPath);
assert(numel(info)==numel(indices),'GIF frame count mismatch.');
fprintf('Saved %d frames: %s\n',numel(info),gifPath);
disp(D.metrics);
end
