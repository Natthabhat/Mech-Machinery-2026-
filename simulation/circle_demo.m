%% circle_demo.m - grab the top of hand V.3, move it out to the largest reachable circle, go round it 3 times
% The circle is the largest horizontal circle (any height) about the device axis on which every shoulder stays
% C.margin inside its rope end-stops. Builds and runs delta_haptic_circle.slx (see run_grab_path below).
% Needs only build_delta_haptic.m; the helpers it uses are local functions at the end of this file:
%   run_grab_path, maxRadius, rodEndTilt, deltaFK, deltaIK
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

%% ===================== local functions =====================

function out = run_grab_path(P, t, xyzT, mdl, live)
% Drive the top of hand V.3 along a prescribed path and let everything else follow.
% Copies the built delta_haptic.slx to <mdl>.slx and adds, inside its Hand subsystem, a 6-DOF joint from World to the
% handle-sphere centre with X/Y/Z prescribed (rotations free, so the parallelogram alone keeps the hand level).
% Motors are unpowered; the joint supplies the "hand" force. Mechanics Explorer plays it in real time when live.
%   t    : Nx1 time (s);  xyzT : Nx3 handle-top path (m, world);  live : real-time pacing on/off
    top = P.handleTop;
    src = 'delta_haptic';
    if bdIsLoaded(mdl), close_system(mdl,0); end
    load_system(fullfile(P.dir,[src '.slx']));
    save_system(src, fullfile(P.dir,[mdl '.slx'])); close_system(src,0); load_system(fullfile(P.dir,[mdl '.slx']));
    assignin(get_param(mdl,'ModelWorkspace'),'target',timeseries(xyzT,t));
    q0 = deltaIK(xyzT(1,:).' - [0;0;top], P);   % assemble at the path start
    assert(all(~isnan(q0)), 'path start is out of reach');
    for i = 1:3
        set_param(sprintf('%s/Leg %d/Upper arm %d/Shoulder%d',mdl,i,i,i),'PositionTargetValue',sprintf('%.6f',q0(i)));
    end

    % grab blocks go inside the Hand subsystem, with their own World Frame (all World Frames are the same world)
    sys = [mdl '/Hand'];
    add = @(src,name,varargin) add_block(src,[sys '/' name],varargin{:});
    add('sm_lib/Frames and Transforms/World Frame','GrabWorld');
    G  = add('sm_lib/Joints/6-DOF Joint','Grab','PxMotionActuationMode','InputMotion','PyMotionActuationMode','InputMotion', ...
             'PzMotionActuationMode','InputMotion','PxTorqueActuationMode','ComputedTorque', ...
             'PyTorqueActuationMode','ComputedTorque','PzTorqueActuationMode','ComputedTorque');
    Tt = add('sm_lib/Frames and Transforms/Rigid Transform','HandleTop','RotationMethod','None', ...
             'TranslationMethod','Cartesian','TranslationCartesianOffset',sprintf('[0 0 %g]',top-0.002));
    add('simulink/Sources/From Workspace','Target','VariableName','target','Interpolate','on','OutputAfterFinalValue','Holding final value');
    add('simulink/Signal Routing/Demux','TargetDemux','Outputs','3');
    add_line(sys,'Target/1','TargetDemux/1');
    con(sys,[sys '/GrabWorld'],'R1',G,'L1'); con(sys,[sys '/HandOffset'],'R1',Tt,'L1'); con(sys,Tt,'R1',G,'R1');
    for k = 1:3
        c = add('nesl_utility/Simulink-PS Converter',sprintf('target_PS%d',k),'Unit','m','FilteringAndDerivatives','filter', ...
                'SimscapeFilterOrder','2','InputFilterTimeConstant','0.001');   % lag ~ 2*tau*speed
        add_line(sys,sprintf('TargetDemux/%d',k),sprintf('target_PS%d/1',k));
        con(sys,c,'R1',G,sprintf('L%d',k+1));
    end
    Simulink.BlockDiagram.arrangeSystem(sys);
    set_param(mdl,'StopTime',num2str(t(end)));
    if live, pace = 'on'; else, pace = 'off'; end
    try set_param(mdl,'EnablePacing',pace,'PacingRate','1'); catch e, warning('Real-time pacing not set: %s', e.message); end
    save_system(mdl);

    out = sim(mdl);
    q   = [out.q1.Data(:) out.q2.Data(:) out.q3.Data(:)];
    xyz = squeeze(out.xyz.Data); if size(xyz,1)==3, xyz = xyz.'; end
    err = max(vecnorm(xyz + [0 0 top] - interp1(t, xyzT, out.xyz.Time), 2, 2));
    fprintf('max handle tracking error %.3g mm\n', 1e3*err);
    fprintf('shoulder range [%.1f %.1f] deg (limits [%.1f %.1f]); motor range [%.0f %.0f] deg\n', min(q(:)), max(q(:)), ...
        P.qMin, P.qMax, min(out.qm1.Data(:)), max(out.qm1.Data(:)));
    assert(err < 1e-3, 'handle did not follow the path');
    tilt = max(rodEndTilt(xyz.', P), [], 'all');
    fprintf('max rod-end tilt %.1f deg (limit %g)\n', tilt, P.tiltMax);
    assert(min(q(:)) > P.qMin && max(q(:)) < P.qMax, 'path drives a shoulder into its rope end-stop');
    assert(tilt < P.tiltMax, 'path tilts a rod end past its misalignment limit');
end

function con(mdl, a, pa, b, pb)
    A = get_param(a,'PortHandles'); B = get_param(b,'PortHandles');
    add_line(mdl, A.([pa(1) 'Conn'])(str2double(pa(2:end))), B.([pb(1) 'Conn'])(str2double(pb(2:end))), 'autorouting','on');
end

function R = maxRadius(P, z, margin, cxy)
% Largest radius (m) of a horizontal circle at height z (rod-end bolt plane, m), centred at cxy, on which every
% shoulder stays inside [qMin+margin, qMax-margin] deg and every rod end stays margin inside P.tiltMax.
% 0 if not even the centre point is reachable.
    th = linspace(0, 2*pi, 181);
    ok = @(r) all(arrayfun(@(a) inRange([cxy(1)+r*cos(a); cxy(2)+r*sin(a); z], P, margin), th));
    R = 0; if ~ok(0), return, end
    lo = 0; hi = 0.25;
    for it = 1:30, m = (lo+hi)/2; if ok(m), lo = m; else, hi = m; end, end
    R = lo;
end

function y = inRange(p, P, margin)
    q = deltaIK(p, P);
    y = all(rodEndTilt(p, P) < P.tiltMax - margin) && all(~isnan(q)) && all(q > P.qMin + margin) && all(q < P.qMax - margin);
end

function tilt = rodEndTilt(p, P)
% Rod-end misalignment (deg) per leg: how far each forearm leans sideways out of its leg plane.
% The rod-end bolts run along the leg tangent v_i at both ends, so tilt_i = asin(|d_i . v_i|) with rod direction
% d_i = (p + r u_i - E_i)/Lb and E_i . v_i = t_i:  tilt_i = asin(|p . v_i - t_i| / Lb). Depends on x, y only.
%   p : 3xN hand positions (rod-end bolt-plane centre, m)  ->  tilt : 3xN (deg)
    v = [-sind(P.phi); cosd(P.phi)].';               % 3x2 leg tangents
    tilt = asind(abs(v*p(1:2,:) - P.t(:)) / P.Lb);
end

function p = deltaFK(qdeg, P)
% Effector centre (rod-end bolt plane) from shoulder angles (q>0 = arm down). Intersects 3 spheres, returns upper solution.
% Per leg: pivot at radius P.R(i), leg plane offset P.t(i) along the tangent, height P.H.
    C = zeros(3,3);
    for i = 1:3
        c = cosd(P.phi(i)); s = sind(P.phi(i));
        rad = P.R(i) + P.La*cosd(qdeg(i)) - P.r;           % radial distance of shifted sphere centre
        C(:,i) = [rad*c - P.t(i)*s; rad*s + P.t(i)*c; P.H - P.La*sind(qdeg(i))];
    end
    % trilateration of equal-radius spheres
    ex = (C(:,2)-C(:,1))/norm(C(:,2)-C(:,1)); a = dot(ex, C(:,3)-C(:,1));
    ey = C(:,3)-C(:,1)-a*ex; ey = ey/norm(ey); ez = cross(ex,ey); d = norm(C(:,2)-C(:,1));
    b = dot(ey, C(:,3)-C(:,1)); x = d/2; y = (a^2+b^2-2*a*x)/(2*b);
    z = sqrt(P.Lb^2 - x^2 - y^2);
    p = C(:,1) + x*ex + y*ey + z*ez; if p(3) < C(3,1), p = C(:,1) + x*ex + y*ey - z*ez; end
end

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
