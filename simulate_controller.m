function R = simulate_controller(type,P,C,options)
%SIMULATE_CONTROLLER simulation function for CABS and CIBS. Might
%not be the most efficient simulation tool since Simulink exists, reason
%why I bothered doing everything in MATLAB is that Simulink gets messy very
%easy and logging all the signal then plotting them gets complicated rather
%quickly 
%
% type = 'CABS' or 'CIBS'
%
% CABS options:
%   options.rho0
%   options.adaptationOn
%
% CIBS requires no additional options.
if nargin < 4, options = struct(); end
type = upper(type);
t = (0:P.Ts:P.tEnd).';
N = numel(t);
x = [P.alpha0; P.q0];
delta = P.delta0;
cfState = [0;0];
chi1 = C.chi10;

if strcmp(type,'CABS')
    if ~isfield(options,'rho0'), options.rho0 = C.CABS.rhoWrong; end
    if ~isfield(options,'adaptationOn'), options.adaptationOn = true; end
    rhoHat = options.rho0;
else
    rhoHat = nan(7,1);
end

% Logs
names = {'alpha','q','alphaRef','alphaRefDot','Az','qDot','qRaw','qc','qcDot', ...
         'z1','zbar1','z2','chi1','deltaRaw','delta','gInner','fHat','nu','deltaInc'};
for i=1:numel(names), log.(names{i}) = nan(N,1); end
log.thetaHat = nan(N,7);
log.rhoHat   = nan(N,7);

for k = 1:N
    meas = plant('outputs',x,delta,P);
    [ar,ardot] = reference(t(k),P);

    z1k = x(1)-ar;
    qRawk = -C.c1*z1k - meas.Az/P.VT + ardot;
    [cfState,qck,qcDotk] = filter_step(cfState,qRawk,C);
    chi1Dot = -C.c1*chi1 + (qck-qRawk);
    zbar1k = z1k-chi1;
    z2k = x(2)-qck;

    switch type
        case 'CABS'
            % Store the parameter estimate actually used to compute this sample's control input.
            rhoHatUsed = rhoHat;
            [deltaRawk,fHat,gInner,thetaHat] = controller_cabs( ...
                'control',x(1),qcDotk,zbar1k,z2k,rhoHatUsed,P,C);
            deltaNew = sat(deltaRawk,P.deltaMin,P.deltaMax);
            isSat = abs(deltaNew-deltaRawk)>1e-12;
            if options.adaptationOn
                rhoDot = controller_cabs('adapt',x(1),deltaNew,z2k,rhoHatUsed,P,C,isSat);
                rhoHat = rhoHatUsed + P.Ts*rhoDot;
                rhoHat = min(max(rhoHat,C.CABS.rhoMin),C.CABS.rhoMax);
            end
            nu = nan; deltaInc = nan;
        case 'CIBS'
            [deltaRawk,deltaInc,gInner,nu] = controller_cibs( ...
                qcDotk,zbar1k,z2k,meas.qDot,delta,P,C);
            deltaNew = sat(deltaRawk,P.deltaMin,P.deltaMax);
            fHat = nan; thetaHat = nan(7,1);
    end

    log.alpha(k)=x(1); log.q(k)=x(2);
    log.alphaRef(k)=ar; log.alphaRefDot(k)=ardot;
    log.Az(k)=meas.Az; log.qDot(k)=meas.qDot;
    log.qRaw(k)=qRawk; log.qc(k)=qck; log.qcDot(k)=qcDotk;
    log.z1(k)=z1k; log.zbar1(k)=zbar1k; log.z2(k)=z2k;
    log.chi1(k)=chi1; log.deltaRaw(k)=deltaRawk; log.delta(k)=deltaNew;
    log.gInner(k)=gInner; log.fHat(k)=fHat; log.nu(k)=nu; log.deltaInc(k)=deltaInc;
    if strcmp(type,'CABS')
        % Log the estimate that generated deltaRawk at this sample.
        log.thetaHat(k,:)=thetaHat.';
        log.rhoHat(k,:)=rhoHatUsed.';
    end
    chi1 = chi1 + P.Ts*chi1Dot;
    x = plant('step',x,deltaNew,P);
    delta = deltaNew;
end

R = log;
R.t = t;
R.controllerType = type;
R.metrics = metrics(R,P,type);
end

function [a,adot] = reference(t,P)
a = P.alphaRefAmp*sin(P.omegaRef*t);
adot = P.alphaRefAmp*P.omegaRef*cos(P.omegaRef*t);
end

function [stateNext,qc,qcDot] = filter_step(state,qRaw,C)
% use the current filter state as the current output, then advance it to k+1.
qc = state(1);
qcDot = state(2);
stateNext = C.Ad_cf*state + C.Bd_cf*qRaw;
end

function y = sat(u,umin,umax)
y = min(max(u,umin),umax);
end

function M = metrics(R,P,type)
eAlpha = R.alpha-R.alphaRef;
eQ = R.q-R.qc;
M.RMSE_alpha_deg = rad2deg(sqrt(mean(eAlpha.^2)));
M.maxAbs_alpha_deg = rad2deg(max(abs(eAlpha)));
M.RMSE_q_deg_s = rad2deg(sqrt(mean(eQ.^2)));
M.maxAbs_q_error_deg_s = rad2deg(max(abs(eQ)));
M.deltaRMS_deg = rad2deg(sqrt(mean(R.delta.^2)));
M.maxAbs_delta_deg = rad2deg(max(abs(R.delta)));
M.maxAbs_deltaRaw_deg = rad2deg(max(abs(R.deltaRaw)));
M.saturationPercent = 100*mean(abs(R.delta)>=0.999*P.deltaMax);
if strcmp(type,'CIBS')
    valid = ~isnan(R.deltaInc);
    M.deltaIncRMS_deg = rad2deg(sqrt(mean(R.deltaInc(valid).^2)));
    M.maxAbs_deltaInc_deg = rad2deg(max(abs(R.deltaInc(valid))));
    eAcc = R.qDot(2:end)-R.nu(1:end-1);
    M.RMSE_qdot_next_deg_s2 = rad2deg(sqrt(mean(eAcc.^2)));
    M.maxAbs_qdot_next_error_deg_s2 = rad2deg(max(abs(eAcc)));
end
end
