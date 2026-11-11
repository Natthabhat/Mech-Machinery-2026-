%% delta_symbolic.m - symbolic kinematics and dynamics of the 3-DOF capstan delta haptic device
% Derives (Symbolic Math Toolbox) and generates MATLAB functions, using the same geometry as build_delta_haptic:
%   loop closure   Phi_i(p,q_i) = |p + r*u_i - E_i(q_i)|^2 - Lb^2 = 0,   E_i = (R_i + La cos q_i) u_i + t_i v_i + (H - La sin q_i) e_z
%   closed-form IK q_i(p),  Jacobians  Phi_p, Phi_q  ->  pdot = J*qdot,  J = -Phi_p \ Phi_q,  J^-1 = -Phi_q \ Phi_p
%   force map      shoulder torque tau_s = J' * F   ->   motor torque tau_m = -tau_s / N   (capstan reverses direction)
%   dynamics       reduced model: each arm = rigid body about its pivot (+ N^2 x motor/small-drum inertia);
%                  each forearm rod split half to the elbow, half to the hand (exact for gravity, approximate for inertia)
%                  (M_q + m_p J'J) qdd = tau_s - G_q(q) - b qd - J'(m_p (beta + g e_z) - F),  pdd = J qdd + beta
% Body masses / centres of mass / inertias are measured from the Simscape model (Inertia Sensors), so they match it.
% p = rod-end bolt-plane centre of hand V.3 (handle top is P.handleTop above it); q in rad, q > 0 = arm down.
% Outputs (this folder): delta_sym_*.m generated functions, delta_dynamics.m / delta_motor_torque.m use them,
%   delta_symbolic.mat (symbolic equations S, measured parameters D).
%   delta_symbolic            % derive, generate, verify against deltaFK/deltaIK and the Simscape model

build_delta_haptic; P = evalin('base','P');
clearvars -except P                       % the build script's variables (e.g. 'subs') would shadow functions here
D = measureBodies(P);                    % masses / COM / inertia from the Simscape bodies
fprintf(['measured: arm %.1f g (COM [%.1f %.1f] mm, I_pivot %.3g kg m^2), motor side %.3g kg m^2, ' ...
         'rod %.1f g, hand %.1f g\n'], 1e3*D.ma, 1e3*D.cxa, 1e3*D.cya, D.Ia, D.Jm, 1e3*D.mr, 1e3*D.mh);

%% ---- symbols ----
q  = sym('q',  [3 1], 'real'); dq = sym('dq', [3 1], 'real');
p  = sym('p',  [3 1], 'real'); dp = sym('dp', [3 1], 'real');
syms H La Lb r N g Ia Jm ma cxa cya mr mh b real
Rl = sym('R', [3 1], 'real'); tl = sym('t', [3 1], 'real');
ez = [0;0;1];
phi = sym(P.phi(:))*sym(pi)/180;          % leg azimuths are exact (180/300/60 deg)

%% ---- kinematics ----
Phi = sym(zeros(3,1)); qIK = sym(zeros(3,1)); E = sym(zeros(3,3));
for i = 1:3
    u = [cos(phi(i)); sin(phi(i)); 0]; v = [-sin(phi(i)); cos(phi(i)); 0];
    E(:,i) = (Rl(i) + La*cos(q(i)))*u + tl(i)*v + (H - La*sin(q(i)))*ez;      % elbow (rod-end eye midpoint)
    A = p + r*u;                                                              % hand attachment point
    Phi(i) = simplify(sum((A - E(:,i)).^2) - Lb^2);
    % closed-form IK: dX*cos q - dZ*sin q = K in the leg frame
    Al = [u.'; v.'; ez.']*A; dX = Rl(i) - Al(1); dY = tl(i) - Al(2); dZ = H - Al(3);
    K  = (Lb^2 - dX^2 - dY^2 - dZ^2 - La^2)/(2*La);
    qIK(i) = -atan2(dZ, dX) - acos(K/sqrt(dX^2 + dZ^2));                     % arms-up branch (checked against deltaFK below)
end
Phi_p = jacobian(Phi, p);                 % 3x3
Phi_q = jacobian(Phi, q);                 % diagonal
assert(isequal(simplify(Phi_q - diag(diag(Phi_q))), sym(zeros(3))), 'Phi_q should be diagonal');
Jinv  = simplify(-diag(1./diag(Phi_q))*Phi_p);                                % qdot = Jinv * pdot
c     = jacobian(Phi_p*dp + Phi_q*dq, [p; q])*[dp; dq];                       % d/dt(Phi_p) pdot + d/dt(Phi_q) qdot

%% ---- dynamics (reduced model) ----
mp  = mh + 3*mr;                                          % hand + half of the 6 rods
Mq  = (Ia + N^2*Jm + mr*La^2)*eye(3);                     % arm + reflected motor side + half of 2 rods at the elbow
V   = sum(ma*g*(H - cxa*sin(q) - cya*cos(q)) + mr*g*(H - La*sin(q)));   % arms + elbow halves (hand: m_p g z, via J')
Gq  = jacobian(V, q).';

S = struct('Phi',Phi,'Phi_p',Phi_p,'Phi_q',Phi_q,'Jinv',Jinv,'c',c,'qIK',qIK,'Mq',Mq,'Gq',Gq,'mp',mp,'V',V,'E',E);
disp('Loop closure, leg 1:'); pretty(Phi(1))
disp('Inverse Jacobian row 1 (qdot_1 = row * pdot):'); pretty(Jinv(1,:))

%% ---- numeric values and generated functions ----
vals = [Rl; tl; H; La; Lb; r; N; g; Ia; Jm; ma; cxa; cya; mr; mh; b];
num  = [P.R(:); P.t(:); P.H; P.La; P.Lb; P.r; P.N; 9.81; D.Ia; D.Jm; D.ma; D.cxa; D.cya; D.mr; D.mh; P.b];
sb   = @(e) subs(e, vals, num);
here = fileparts(mfilename('fullpath'));
gen  = @(e, name, vars) matlabFunction(sb(e), 'File', fullfile(here, name), 'Vars', vars, 'Optimize', true);
gen(Phi,   'delta_sym_Phi',   {p, q});
gen(Phi_p, 'delta_sym_Phi_p', {p, q});
gen(diag(Phi_q), 'delta_sym_Phi_q', {p, q});        % diagonal entries
gen(c,     'delta_sym_c',     {p, q, dp, dq});
gen(qIK,   'delta_sym_IK',    {p});
gen(Gq,    'delta_sym_G',     {q});
dyn.Mq = double(sb(Mq(1,1))); dyn.mp = double(sb(mp)); dyn.N = P.N; dyn.b = P.b; dyn.g = 9.81;
save(fullfile(here, 'delta_symbolic.mat'), 'S', 'D', 'dyn', 'P');
clear delta_dynamics delta_motor_torque delta_kinematics   % reload the new parameters on next call
fprintf('generated delta_sym_{Phi,Phi_p,Phi_q,c,IK,G}.m; M_q = %.4g kg m^2 per arm, m_p = %.1f g\n', dyn.Mq, 1e3*dyn.mp);

%% ---- verification ----
rng(1); ok = 0; eJ = 0; eIK = 0; ePhi = 0;
for k = 1:200
    qd = (P.qMid + (rand(1,3) - 0.5)*60);                 % deg, around the rope-centred pose
    pk = deltaFK(qd, P);
    if any(~isreal(pk)), continue, end
    ok = ok + 1;
    ePhi = max(ePhi, norm(delta_sym_Phi(pk, qd(:)*pi/180)));
    eIK  = max(eIK, max(abs(mod(delta_sym_IK(pk) - qd(:)*pi/180 + pi, 2*pi) - pi)));
    [~, J] = delta_dynamics(qd(:)*pi/180, [0;0;0], [0;0;0], [0;0;0]);
    h = 1e-6; Jfd = zeros(3);
    for j = 1:3, e = zeros(1,3); e(j) = h*180/pi; Jfd(:,j) = (deltaFK(qd + e, P) - deltaFK(qd - e, P))/(2*h); end
    eJ = max(eJ, norm(J - Jfd)/norm(Jfd));
end
fprintf('over %d random poses: |Phi(FK(q))| <= %.2g, |IK(FK(q)) - q| <= %.2g rad, J vs finite differences <= %.2g (rel)\n', ...
        ok, ePhi, eIK, eJ);
assert(ePhi < 1e-12 && eIK < 1e-9 && eJ < 1e-5, 'symbolic kinematics disagree with deltaFK/deltaIK');
checkVelAcc(P);


% Simscape check at its assembly pose (P.q0, rest): starting acceleration for zero motor torque (free fall) and
% for the symbolic gravity-compensation torque. Exact statics -> zero acceleration; free fall checks the mass model.
q0 = P.q0*[1 1 1]*pi/180;
taum = delta_motor_torque(q0(:), [0;0;0], [0;0;0], [0;0;0]);
mdl = 'delta_haptic'; load_system(fullfile(P.dir, [mdl '.slx']));   % save_system(...,newName) in measureBodies renamed it
acc = @(tv) simStartAcc(mdl, tv, q0(:));
a0 = acc([0;0;0]); ag = acc(taum);
set_param([mdl '/MotorTorque'], 'Value', '[0;0;0]');
qdd = delta_dynamics(q0(:), [0;0;0], [0;0;0], [0;0;0]);
fprintf('gravity compensation torque [%s] mN*m: Simscape start acceleration %.3g rad/s^2 = %.2f %% of free fall (%.3g)\n', ...
        num2str(1e3*taum.', '%8.3f'), norm(ag), 100*norm(ag)/norm(a0), norm(a0));
fprintf('free-fall start acceleration: symbolic [%s], Simscape [%s] rad/s^2 (%.1f %% difference)\n', ...
        num2str(qdd.', '%8.2f'), num2str(a0.', '%8.2f'), 100*norm(qdd - a0)/norm(a0));
assert(norm(ag) < 0.005*norm(a0), 'symbolic gravity compensation does not hold the Simscape model');
assert(norm(qdd - a0) < 0.05*norm(a0), 'reduced dynamic model is off by more than 5 %');

%% ---- helpers ----
function a = simStartAcc(mdl, tv, q0)
    % shoulder accelerations (rad/s^2) at t=0 from rest, constant motor torque tv: quadratic fit over the first 10 ms
    set_param([mdl '/MotorTorque'], 'Value', mat2str(tv, 10));
    o = sim(mdl, 'StopTime', '0.01', 'MaxStep', '1e-4');
    t = o.q1.Time; q = [o.q1.Data(:) o.q2.Data(:) o.q3.Data(:)]*pi/180; a = zeros(3,1);
    for j = 1:3, c = polyfit(t, q(:,j) - q0(j), 2); a(j) = 2*c(1); end
end
function D = measureBodies(P)
    % mass, COM and centred inertia of one arm, motor shaft (+ small drum), rod and hand, from Inertia Sensors
    src = 'delta_haptic'; m = 'delta_haptic_props';
    if bdIsLoaded(m), close_system(m, 0); end
    save_system(src, fullfile(P.dir, [m '.slx'])); load_system(fullfile(P.dir, [m '.slx']));
    probes = {'arm',   'Leg 1/Upper arm 1', 'Shoulder1',  'R1';
              'motor', 'Leg 1/Drive 1',     'MotorShaft1','R1';
              'rod',   'Leg 1/Forearm 1a',  'RodEndA1a',  'R1';
              'hand',  'Hand',              'HandOffset', 'L1'};
    for k = 1:size(probes, 1)
        sys = [m '/' probes{k,2}]; nm = probes{k,1};
        s = add_block('sm_lib/Body Elements/Inertia Sensor', [sys '/IS_' nm], 'SenseMass', 'on', 'SenseCenterOfMass', 'on', ...
                      'SenseCenteredInertiaMatrix', 'on');
        a = get_param([sys '/' probes{k,3}], 'PortHandles'); b = get_param(s, 'PortHandles');
        add_line(sys, a.([probes{k,4}(1) 'Conn'])(str2double(probes{k,4}(2))), b.LConn(1));
        for j = 1:numel(b.RConn)
            c = add_block('nesl_utility/PS-Simulink Converter', sprintf('%s/IS_%s_ps%d', sys, nm, j));
            w = add_block('simulink/Sinks/To Workspace', sprintf('%s/IS_%s_out%d', sys, nm, j), ...
                          'VariableName', sprintf('IS_%s_%d', nm, j), 'SaveFormat', 'Array');
            cp = get_param(c, 'PortHandles'); wp = get_param(w, 'PortHandles');
            add_line(sys, b.RConn(j), cp.LConn(1)); add_line(sys, cp.Outport(1), wp.Inport(1));
        end
    end
    o = sim(m, 'StopTime', '0'); close_system(m, 0); delete(fullfile(P.dir, [m '.slx']));
    for k = 1:size(probes, 1)
        nm = probes{k,1}; vals = {};
        for j = 1:3, vals{j} = squeeze(o.(sprintf('IS_%s_%d', nm, j))); vals{j} = vals{j}(:, :, end); end %#ok<AGROW>
        sz = cellfun(@numel, vals);
        R.(nm).m = vals{sz == 1}; cm = vals{sz == 3}; R.(nm).com = cm(:, end); R.(nm).Ic = vals{sz == 9};
    end
    % arm: COM in the shoulder follower frame (x along the arm, y down at q=0, z = pivot axis)
    D.ma = R.arm.m; D.cxa = R.arm.com(1); D.cya = R.arm.com(2);
    D.Ia = R.arm.Ic(3,3) + D.ma*(D.cxa^2 + D.cya^2);
    D.Jm = R.motor.Ic(3,3) + R.motor.m*sum(R.motor.com(1:2).^2);   % small drum + rotor block (P.Jrot), about the shaft
    D.mr = R.rod.m; D.mh = R.hand.m; D.raw = R;
    assert(abs(R.rod.com(3) + P.Lb/2) < 1e-3, 'rod COM is not at mid-length (half/half split assumption)');
end

function checkVelAcc(P)
    % velocity / acceleration: finite differences of p(t) along a smooth joint trajectory, and the inverse round trip
    qf = @(t) (P.qMid + [20 -15 10].*sin([3 4 5]*t + [0 1 2])).'*pi/180;   % rad
    dqf = @(t) ([20 -15 10].*[3 4 5].*cos([3 4 5]*t + [0 1 2])).'*pi/180;
    ddqf = @(t) (-[20 -15 10].*[3 4 5].^2.*sin([3 4 5]*t + [0 1 2])).'*pi/180;
    h = 1e-4; ev = 0; ea = 0; ei = 0;
    for t = linspace(0, 2, 41)
        [p, dp, ddp] = delta_kinematics(qf(t), dqf(t), ddqf(t));
        pm = delta_kinematics(qf(t-h)); pp = delta_kinematics(qf(t+h));
        ev = max(ev, norm(dp - (pp - pm)/(2*h))/max(norm(dp), 1e-3));
        ea = max(ea, norm(ddp - (pp - 2*p + pm)/h^2)/max(norm(ddp), 1e-2));
        [qi, dqi, ddqi] = delta_kinematics_inv(p, dp, ddp);
        ei = max(ei, norm([qi - qf(t); dqi - dqf(t); ddqi - ddqf(t)]));
    end
    fprintf('velocity vs finite differences <= %.2g (rel), acceleration <= %.2g (rel), inverse round trip <= %.2g\n', ev, ea, ei);
    assert(ev < 1e-6 && ea < 1e-4 && ei < 1e-9, 'velocity / acceleration kinematics are wrong');
end
