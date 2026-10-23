function varargout = controller_cabs(action,varargin)
%CONTROLLER_CABS CABS control and adaptation in one module.
%
% control:
% [deltaRaw,fHat,gHat,thetaHat] = controller_cabs('control',...)
%
% adapt:
% rhoDot = controller_cabs('adapt',...)
switch lower(action)
    case 'control'
        alpha  = varargin{1};
        qcDot  = varargin{2};
        zbar1  = varargin{3};
        z2     = varargin{4};
        rhoHat = varargin{5};
        P      = varargin{6};
        C      = varargin{7};

        thetaHat = C.CABS.thetaNom .* rhoHat;
        [Yf,Yg] = regressors(alpha,P,0);
        fHat = Yf*thetaHat(1:5);
        gHat = Yg*thetaHat(6:7);
        gHat = guard_gain(gHat,C.gHatMin);
        deltaRaw = (qcDot - C.c2*z2 - zbar1 - fHat)/gHat;
        varargout = {deltaRaw,fHat,gHat,thetaHat};
    case 'adapt'
        alpha       = varargin{1};
        deltaApplied= varargin{2};
        z2          = varargin{3};
        rhoHat      = varargin{4};
        P           = varargin{5};
        C           = varargin{6};
        isSaturated = varargin{7};

        if C.CABS.freezeOnSaturation && isSaturated
            varargout{1} = zeros(7,1);
            return;
        end

        [~,~,Yfull] = regressors(alpha,P,deltaApplied);
        Psi = Yfull .* C.CABS.thetaNom.';
        rhoDot = C.CABS.Gamma * Psi.' * z2;

        % Componentwise box projection.
        for i = 1:7
            if rhoHat(i) <= C.CABS.rhoMin(i) && rhoDot(i) < 0
                rhoDot(i) = 0;
            elseif rhoHat(i) >= C.CABS.rhoMax(i) && rhoDot(i) > 0
                rhoDot(i) = 0;
            end
        end
        varargout{1} = rhoDot;
end
end

function [Yf,Yg,Yfull] = regressors(alpha,P,delta)
aa = alpha*abs(alpha);
Yf = P.Kq*[alpha^3, aa, alpha, P.M*aa, P.M*alpha];
Yg = P.Kq*[P.M, 1];
Yfull = [Yf, Yg*delta];
end

function g = guard_gain(g,gMin)
if abs(g) < gMin
    if g >= 0, g = gMin; else, g = -gMin; end
end
end
