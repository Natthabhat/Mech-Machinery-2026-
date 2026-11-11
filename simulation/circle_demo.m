%% circle_demo.m - grab the top of hand V.3, move it out to the largest reachable circle, go round it 3 times
% The circle is the largest horizontal circle (any height) about the device axis on which every shoulder stays
% C.margin inside its rope end-stops. Builds and runs delta_haptic_circle.slx (see run_grab_path).
%   circle_demo            % build + run with live 3D view
%   circle_demo(false)     % build + run without pacing (batch check)
function out = circle_demo(live)
if nargin < 1, live = true; end
build_delta_haptic; P = evalin('base','P');

% path: ramp out C.Tout s, C.nRev revolutions of C.Trev s each, ramp back C.Tout s
C.margin = 3;                            % deg kept clear of each rope end-stop
C.nRev = 3; C.Trev = 5; C.Tout = 2;
p0 = deltaFK(P.qMid*[1 1 1], P); zs = p0(3) + (-0.08:0.0025:0.08);
Rz = arrayfun(@(z) maxRadius(P, z, C.margin, p0(1:2)), zs);
[C.R, k] = max(Rz); assert(C.R > 0.01, 'no reachable circle found');
C.c = [p0(1); p0(2); zs(k) + P.handleTop];   % circle centre at the handle top
T  = 2*C.Tout + C.nRev*C.Trev; t = (0:0.01:T)';
s01 = min(min([t, T-t]/C.Tout, [], 2), 1); rr = C.R*(3*s01.^2 - 2*s01.^3);   % smoothstep out and back
th = 2*pi*C.nRev*min(max(t - C.Tout, 0), C.nRev*C.Trev)/(C.nRev*C.Trev);    % 0 .. nRev*360 deg
xyzT = [C.c(1)+rr.*cos(th), C.c(2)+rr.*sin(th), C.c(3)+0*t];
assert(rr(1) == 0 && rr(end) == 0, 'path must start and end at the circle centre');
fprintf('largest circle: centre [%s] mm (handle top), radius %.1f mm, %d revolutions in %g s\n', ...
    num2str(1e3*C.c.','%7.1f'), 1e3*C.R, C.nRev, T);
out = run_grab_path(P, t, xyzT, 'delta_haptic_circle', live);
end
