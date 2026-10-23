function [deltaRaw,deltaInc,gCtrl,nu] = controller_cibs(qcDot,zbar1,z2,qDotMeasured,deltaPrev,P,C)
%CONTROLLER_CIBS Incremental inner-loop control law.
gCtrl = P.Kq*(C.CIBS.thetaCtrl(6)*P.M + C.CIBS.thetaCtrl(7));
if abs(gCtrl) < C.gHatMin
    if gCtrl >= 0, gCtrl = C.gHatMin; else, gCtrl = -C.gHatMin; end
end
nu = qcDot - C.c2*z2 - zbar1;
deltaInc = (nu - qDotMeasured)/gCtrl;
deltaRaw = deltaPrev + deltaInc;
end
