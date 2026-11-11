%% helix_demo.m - grab the top of hand V.3 and trace the outer edge of the reachable workspace
% Moves out to the largest reachable circle, then in each of 3 revolutions spirals down to the lowest height where a
% circle (radius >= C.Rmin) is still reachable, up to the highest, and back to the start height; at every height the
% radius is the largest one with every shoulder C.margin inside its rope end-stops. Builds delta_haptic_helix.slx.
%   helix_demo            % build + run with live 3D view
%   helix_demo(false)     % build + run without pacing (batch check)
function out = helix_demo(live)
if nargin < 1, live = true; end
build_delta_haptic; P = evalin('base','P');

C.margin = 0.5;          % deg kept clear of each rope end-stop (0 would push the arms into the stop springs)
C.Rmin   = 0.001;        % m, smallest circle still counted as "circling" at the lowest/highest point
C.nRev = 3; C.Trev = 7; C.Tout = 2;    % 7 s per revolution; each one goes lowest -> highest

% workspace edge: largest radius at each height (rod-end bolt plane), about the device axis
p0 = deltaFK(P.qMid*[1 1 1], P);
zs = p0(3) + (-0.20:0.002:0.20);
Rz = arrayfun(@(z) maxRadius(P, z, C.margin, p0(1:2)), zs);
okz = find(Rz >= C.Rmin); assert(~isempty(okz), 'no reachable circle found');
assert(okz(1) > 1 && okz(end) < numel(zs), 'height search range too small: widen zs');
% refine the lowest / highest height to 0.01 mm by bisection on "a circle of C.Rmin fits"
fits = @(z) maxRadius(P, z, C.margin, p0(1:2)) >= C.Rmin;
ends = [zs(okz(1)-1) zs(okz(1)); zs(okz(end)+1) zs(okz(end))];   % [outside inside] per end
for e = 1:2
    while abs(diff(ends(e,:))) > 1e-5
        m = mean(ends(e,:)); if fits(m), ends(e,2) = m; else, ends(e,1) = m; end
    end
end
zs = [ends(1,2), zs(okz(1):okz(end)), ends(2,2)];
Rz = [maxRadius(P, ends(1,2), C.margin, p0(1:2)), Rz(okz(1):okz(end)), maxRadius(P, ends(2,2), C.margin, p0(1:2))];
[Rbig, k] = max(Rz); zMid = zs(k); zLo = zs(1); zHi = zs(end);
Rof = @(z) interp1(zs, Rz, z, 'pchip');

% height in every revolution: start -> lowest -> highest -> start, smoothstep between
T  = 2*C.Tout + C.nRev*C.Trev; t = (0:0.01:T)';
u  = min(max((t - C.Tout)/(C.nRev*C.Trev), 0), 1);          % 0..1 over the revolutions
w  = C.nRev*u - min(floor(C.nRev*u), C.nRev - 1);            % 0..1 within the current revolution
ss = @(x) 3*x.^2 - 2*x.^3;
z  = zMid + (zLo - zMid)*ss(min(3*w, 1));
z  = z + (zHi - zLo)*ss(min(max(3*w - 1, 0), 1));
z  = z + (zMid - zHi)*ss(min(max(3*w - 2, 0), 1));
k01 = ss(min(min([t, T-t]/C.Tout, [], 2), 1));               % radius ramp out / back at the start height
rr = k01 .* Rof(z);
th = 2*pi*C.nRev*u;
xyzT = [p0(1)+rr.*cos(th), p0(2)+rr.*sin(th), z + P.handleTop];
assert(rr(1) == 0 && rr(end) == 0, 'path must start and end on the device axis');

% the path itself must stay inside the rope limits (it runs along the edge, C.margin inside it)
qPath = cell2mat(arrayfun(@(k) deltaIK(xyzT(k,:).' - [0;0;P.handleTop], P), (1:numel(t))', 'UniformOutput', false));
assert(all(~isnan(qPath(:))) && all(qPath(:) > P.qMin) && all(qPath(:) < P.qMax), 'helix path leaves the rope limits');

fprintf('workspace edge: largest circle %.1f mm at handle height %.1f mm\n', 1e3*Rbig, 1e3*(zMid + P.handleTop));
fprintf('lowest %.1f mm (radius %.1f), highest %.1f mm (radius %.1f), %d revolutions in %g s\n', ...
    1e3*(zLo + P.handleTop), 1e3*Rz(1), 1e3*(zHi + P.handleTop), 1e3*Rz(end), C.nRev, T);
out = run_grab_path(P, t, xyzT, 'delta_haptic_helix', live);
end
