function [qdd, J, p, pdd] = delta_dynamics(q, dq, tau_m, F)
% Forward dynamics of the delta (reduced model from delta_symbolic.m; run that first).
%   q, dq  : shoulder angles / rates (3x1, rad, rad/s; q > 0 = arm down)
%   tau_m  : motor shaft torques (3x1, N*m)           F : external force on the hand (3x1, N, world)
% Returns joint accelerations qdd, Jacobian J (pdot = J*qdot), hand position p (rod-end bolt plane, m) and pdd.
    persistent K
    if isempty(K), K = load(fullfile(fileparts(mfilename('fullpath')), 'delta_symbolic.mat'), 'dyn', 'P'); end
    d = K.dyn; ez = [0;0;1];
    p   = deltaFK(q(:).'*180/pi, K.P);
    Pp  = delta_sym_Phi_p(p, q(:));  Pq = diag(delta_sym_Phi_q(p, q(:)));
    J   = -Pp \ Pq;  dp = J*dq(:);
    beta = -Pp \ delta_sym_c(p, q(:), dp, dq(:));      % pdd = J*qdd + beta
    tau_s = -d.N*tau_m(:);                             % capstan: motor and arm turn opposite ways
    M   = d.Mq*eye(3) + d.mp*(J.'*J);
    qdd = M \ (tau_s - delta_sym_G(q(:)) - d.b*dq(:) - J.'*(d.mp*(beta + d.g*ez) - F(:)));
    pdd = J*qdd + beta;
end
