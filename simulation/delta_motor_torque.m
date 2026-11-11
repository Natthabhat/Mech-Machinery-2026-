function [tau_m, J] = delta_motor_torque(q, dq, qdd, Fhand)
% Motor torques for a motion, with gravity compensation and haptic force rendering (reduced model, delta_symbolic.m).
%   q, dq, qdd : shoulder angles / rates / accelerations (3x1, rad, q > 0 = arm down); use zeros for statics
%   Fhand      : force the hand should push on the user (3x1, N, world), e.g. [0;0;-1] renders 1 N downward
% Returns motor shaft torques tau_m (3x1, N*m) and the Jacobian J (pdot = J*qdot).
%   Statics:  tau_m = -( G(q) + J'*(m_p*g*e_z + Fhand) ) / N
    persistent K
    if isempty(K), K = load(fullfile(fileparts(mfilename('fullpath')), 'delta_symbolic.mat'), 'dyn', 'P'); end
    d = K.dyn; ez = [0;0;1];
    p   = deltaFK(q(:).'*180/pi, K.P);
    Pp  = delta_sym_Phi_p(p, q(:));  Pq = diag(delta_sym_Phi_q(p, q(:)));
    J   = -Pp \ Pq;  dp = J*dq(:);
    beta = -Pp \ delta_sym_c(p, q(:), dp, dq(:));
    tau_s = d.Mq*qdd(:) + delta_sym_G(q(:)) + d.b*dq(:) + J.'*(d.mp*(J*qdd(:) + beta + d.g*ez) + Fhand(:));
    tau_m = -tau_s/d.N;
end
