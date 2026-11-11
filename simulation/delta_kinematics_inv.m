function [q, dq, ddq, Jinv] = delta_kinematics_inv(p, dp, ddp)
% Inverse position / velocity / acceleration kinematics of the delta (from delta_symbolic.m; run that first).
%   p, dp, ddp : hand position, velocity, acceleration (3x1; m, m/s, m/s^2; rod-end bolt-plane centre, world)
%   q, dq, ddq : shoulder angles, rates, accelerations (3x1; rad, rad/s, rad/s^2; q > 0 = arm down)
%   Jinv       : inverse Jacobian, dq = Jinv*dp  (closed form: Phi_q is diagonal)
% Position:      closed-form IK (arms-up branch), NaN where out of reach
% Velocity:      dq  = -Phi_q \ (Phi_p*dp)
% Acceleration:  ddq = -Phi_q \ (Phi_p*ddp + c(p,q,dp,dq))
    if nargin < 2, dp = zeros(3,1); end
    if nargin < 3, ddp = zeros(3,1); end
    p = p(:); dp = dp(:); ddp = ddp(:);
    q   = delta_sym_IK(p);
    if any(~isfinite(q)) || any(imag(q) ~= 0), q(:) = NaN; dq = NaN(3,1); ddq = NaN(3,1); Jinv = NaN(3); return, end
    Pp  = delta_sym_Phi_p(p, q);  pq = delta_sym_Phi_q(p, q);
    Jinv = -Pp ./ pq;                        % row i divided by dPhi_i/dq_i
    dq  = Jinv*dp;
    ddq = -(Pp*ddp + delta_sym_c(p, q, dp, dq)) ./ pq;
end
