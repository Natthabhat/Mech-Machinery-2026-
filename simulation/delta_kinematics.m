function [p, dp, ddp, J] = delta_kinematics(q, dq, ddq)
% Forward position / velocity / acceleration kinematics of the delta (from delta_symbolic.m; run that first).
%   q, dq, ddq : shoulder angles, rates, accelerations (3x1; rad, rad/s, rad/s^2; q > 0 = arm down)
%   p, dp, ddp : hand position, velocity, acceleration (3x1; m, m/s, m/s^2; rod-end bolt-plane centre, world)
%   J          : Jacobian, dp = J*dq
% Velocity:      Phi_p*dp + Phi_q*dq = 0                  ->  dp  = J*dq,  J = -Phi_p \ Phi_q
% Acceleration:  Phi_p*ddp + Phi_q*ddq + c(p,q,dp,dq) = 0  ->  ddp = J*ddq - Phi_p \ c
    if nargin < 2, dq = zeros(3,1); end
    if nargin < 3, ddq = zeros(3,1); end
    persistent P
    if isempty(P), K = load(fullfile(fileparts(mfilename('fullpath')), 'delta_symbolic.mat'), 'P'); P = K.P; end
    q = q(:); dq = dq(:); ddq = ddq(:);
    p   = deltaFK(q.'*180/pi, P);
    Pp  = delta_sym_Phi_p(p, q);  Pq = diag(delta_sym_Phi_q(p, q));
    J   = -Pp \ Pq;
    dp  = J*dq;
    ddp = J*ddq - Pp \ delta_sym_c(p, q, dp, dq);
end
