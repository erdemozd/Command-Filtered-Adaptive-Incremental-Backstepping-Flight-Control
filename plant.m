function varargout = plant(action,varargin)
%PLANT Nonlinear aircraft model and fixed-step propagation.
%   meas  = plant('outputs',x,delta,P)
%   xNext = plant('step',x,delta,P)
switch lower(action)
    case 'outputs'
        x = varargin{1}; delta = varargin{2}; P = varargin{3};
        varargout{1} = outputs(x,delta,P);
    case 'step'
        x = varargin{1}; delta = varargin{2}; P = varargin{3};
        varargout{1} = rk4step(x,delta,P);
end
end

function out = outputs(x,delta,P)
alpha = x(1);
q     = x(2);
% Normal-force aerodynamics are measured through Az.
aa = alpha*abs(alpha);
phiZ1 = -288.7*alpha^3 + 50.32*aa - 23.89*alpha;
phiZ2 = -13.53*aa + 4.185*alpha;
bz = 1.6238*P.M - 6.7240;
Cz = phiZ1 + P.M*phiZ2;
% Pitch aerodynamics use the experiment-specific actual plant parameters.
th = P.thetaPlant;
Cm = th(1)*alpha^3 + th(2)*aa + th(3)*alpha + ...
     th(4)*P.M*aa + th(5)*P.M*alpha;
bm = th(6)*P.M + th(7);
Az = (P.qbar*P.S/P.m)*(Cz + bz*delta);
alphaDot = q + Az/P.VT;
qDot = P.Kq*(Cm + bm*delta);

out.alphaDot = alphaDot;
out.qDot = qDot;
out.Az = Az;
out.Cz = Cz;
out.Cm = Cm;
out.bz = bz;
out.bm = bm;
end

function dx = deriv(x,delta,P)
o = outputs(x,delta,P);
dx = [o.alphaDot; o.qDot];
end

function xNext = rk4step(x,delta,P)
h = P.Ts;
k1 = deriv(x,delta,P);
k2 = deriv(x+0.5*h*k1,delta,P);
k3 = deriv(x+0.5*h*k2,delta,P);
k4 = deriv(x+h*k3,delta,P);
xNext = x + h*(k1 + 2*k2 + 2*k3 + k4)/6;
end
