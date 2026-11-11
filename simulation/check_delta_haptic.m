%% check_delta_haptic.m - build, simulate (free fall under gravity), compare to analytic FK
build_delta_haptic;

% screw-hole check, bracket foot face (y=24.1 mm) mapped to the world with the same frame chain as the model:
% front round holes (x=±30.6, z=+1) must sit on the base front holes; base rear holes must lie on the slot (x=±16.6, z=-55..-42)
foot = @(i, xb, zb) bracketToWorld(P, i, xb, zb);
wf = 0; wr = 0;
for i = 1:3
    for xb = [-30.6 30.6]
        w = foot(i, xb, 1e3*P.zHole);
        wf = max(wf, min(vecnorm(P.holesFront{i}*1e-3 - w(1:2).', 2, 2)));
        assert(abs(w(3) - 0.010) < 1e-9, 'bracket foot not on base top');
    end
    for k = 1:2
        h = P.holesRear{i}(k,:).'*1e-3; d = inf;
        for xb = [-16.6 16.6]
            a = foot(i, xb, 1e3*P.slot(1)); b = foot(i, xb, 1e3*P.slot(2)); a = a(1:2); b = b(1:2);
            s = min(max(dot(h-a, b-a)/dot(b-a, b-a), 0), 1); d = min(d, norm(a + s*(b-a) - h));
        end
        wr = max(wr, d);
    end
end
fprintf('front screw holes: max offset %.3g mm; rear holes: max distance from slot centre line %.3g mm\n', 1e3*wf, 1e3*wr);
assert(wf < 5e-5 && wr < 5e-5, 'bracket screw holes do not line up with the base');   % hole data is rounded to 0.01 mm
out = sim('delta_haptic','StopTime','1');
q   = [out.q1.Data(:) out.q2.Data(:) out.q3.Data(:)];          % deg
xyz = squeeze(out.xyz.Data); if size(xyz,1)==3, xyz = xyz.'; end % m
err = zeros(size(q,1),1);
for k = 1:size(q,1), err(k) = norm(deltaFK(q(k,:),P) - xyz(k,:).'); end
fprintf('q start [%s] deg, end [%s] deg\n', num2str(q(1,:),'%7.2f'), num2str(q(end,:),'%7.2f'));
fprintf('effector start [%s] mm, end [%s] mm\n', num2str(1e3*xyz(1,:),'%7.1f'), num2str(1e3*xyz(end,:),'%7.1f'));
fprintf('max |Simscape - analytic FK| = %.3g mm\n', 1e3*max(err));
assert(max(err) < 1e-4, 'Simscape kinematics disagree with analytic FK');

% end-stop check: drive all motors hard (arm down, then up) and confirm the rope limits hold
for tau = [0.3 -0.3]
    set_param('delta_haptic/MotorTorque','Value',sprintf('[%g;%g;%g]',tau,tau,tau));
    o = sim('delta_haptic','StopTime','1');
    qq = [o.q1.Data(:) o.q2.Data(:) o.q3.Data(:)];
    fprintf('motor %+g N*m -> q range [%.1f %.1f] deg (limits [%.1f %.1f])\n', tau, min(qq(:)), max(qq(:)), P.qMin, P.qMax);
    assert(min(qq(:)) > P.qMin-3 && max(qq(:)) < P.qMax+3, 'end-stop exceeded by >3 deg');
end
set_param('delta_haptic/MotorTorque','Value','[0;0;0]');

% IK round trip: FK(IK(p)) = p over a grid of hand positions around the rope-centred pose
p0 = deltaFK(P.qMid*[1 1 1], P); e = 0;
for dx = -0.04:0.02:0.04, for dy = -0.04:0.02:0.04, for dz = -0.04:0.02:0.04
    p = p0 + [dx; dy; dz]; e = max(e, norm(deltaFK(deltaIK(p, P), P) - p));
end, end, end
fprintf('max |FK(IK(p)) - p| = %.3g mm\n', 1e3*e);
assert(e < 1e-9, 'deltaIK does not invert deltaFK');

function w = bracketToWorld(P, i, xb, zb)
    % bracket-frame foot point (mm; foot face y=24.1) -> world (m), same chain as the model's ShoulderFrame/BracketFrame
    Rz = [cosd(P.phi(i)) -sind(P.phi(i)) 0; sind(P.phi(i)) cosd(P.phi(i)) 0; 0 0 1];
    pb = [xb; 24.1; zb]*1e-3 + [0; 0.090; -P.zArm];
    w  = Rz*([P.R(i); P.t(i); P.H] + [1 0 0; 0 0 1; 0 -1 0]*pb);
end
