function q = deltaIK(p, P)
% Shoulder angles (deg, q>0 = arm down) that put the effector centre (rod-end bolt plane) at p (3x1, m).
% Per leg: elbow E(q) = [R + La*cos q, t, H - La*sin q] in the leg frame must be Lb from the hand attachment point.
% Of the two solutions, returns the arms-up one (nearest P.qMid). NaN where p is out of reach.
    q = nan(1,3);
    for i = 1:3
        c = cosd(P.phi(i)); s = sind(P.phi(i));
        A = [c s 0; -s c 0; 0 0 1]*p + [P.r; 0; 0];        % attachment point in the leg frame
        dx = P.R(i) - A(1); dy = P.t(i) - A(2); dz = P.H - A(3);
        K = (P.Lb^2 - dx^2 - dy^2 - dz^2 - P.La^2)/(2*P.La); % dx*cos q - dz*sin q = K
        rho = hypot(dx, dz); if abs(K) > rho, continue, end
        al = atan2d(dz, dx); sol = -al + [1 -1]*acosd(K/rho);
        sol = mod(sol - P.qMid + 180, 360) - 180 + P.qMid;   % wrap near the working range
        [~, k] = min(abs(sol - P.qMid)); q(i) = sol(k);
    end
end
