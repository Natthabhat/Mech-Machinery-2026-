function tilt = rodEndTilt(p, P)
% Rod-end misalignment (deg) per leg: how far each forearm leans sideways out of its leg plane.
% The rod-end bolts run along the leg tangent v_i at both ends, so tilt_i = asin(|d_i . v_i|) with rod direction
% d_i = (p + r u_i - E_i)/Lb and E_i . v_i = t_i:  tilt_i = asin(|p . v_i - t_i| / Lb). Depends on x, y only.
%   p : 3xN hand positions (rod-end bolt-plane centre, m)  ->  tilt : 3xN (deg)
    v = [-sind(P.phi); cosd(P.phi)].';               % 3x2 leg tangents
    tilt = asind(abs(v*p(1:2,:) - P.t(:)) / P.Lb);
end
