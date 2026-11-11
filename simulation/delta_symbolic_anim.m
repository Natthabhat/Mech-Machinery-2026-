%% delta_symbolic_anim.m - symbolic kinematics of the delta haptic device + animation (fanimator)
% The hand follows a known path p(t). For each leg, solve() gives the shoulder angle q_i from the
% forearm-length equation, diff() gives velocity and acceleration, fanimator/playAnimation animates it.
%
% Picture of one leg (seen from the side, leg i):
%
%        C_i  o------- hand (moves, stays level)
%            /
%           /  forearm, length Lb (2 parallel rods, Lb = 170 mm)
%          /
%    B_i  o  elbow
%         \
%          \  upper arm, length La, angle q_i
%           \
%      A_i   o  shoulder pivot (fixed, on the motor bracket)
%
% Units: metres, radians, seconds (plots in degrees).
% Sign convention (same as deltaIK/deltaFK): q_i > 0 = arm turned down, q_i < 0 = arm up.
% World frame: z up, origin at the base centre on the base top.

clear; close all
% P = all device parameters, made by build_delta_haptic.m and saved by delta_symbolic.m
K = load(fullfile(fileparts(mfilename('fullpath')), 'delta_symbolic.mat'), 'P'); P = K.P;

%% Symbolic variables
t = sym('t', 'real');                 % t      : time (s)
q = sym('q', [3 1], 'real');          % q1..q3 : shoulder angle of leg 1, 2, 3 (rad)
syms x y z real                       % x y z  : hand position = centre of the hand's rod-end bolt plane (m)
p = [x; y; z];                        % p      : hand position vector

%% Parameters (from the CAD, see PROJECT_NOTES.md)
R   = P.R;            % R(i)  : base centre -> shoulder pivot axis, radial distance of leg i (m, ~79-81 mm)
T   = P.t;            % T(i)  : small sideways offset of leg i along its pivot axis (m, < 0.5 mm)
H   = P.H;            % H     : height of the shoulder pivots above the base top (m, 124.1 mm)
La  = P.La;           % La    : upper arm, shoulder pivot -> elbow (m, 72.1 mm)
Lb  = P.Lb;           % Lb    : forearm, rod-end eye centre -> eye centre (m, 170 mm)
r   = P.r;            % r     : hand centre -> hand rod-end bolt line (m, 34 mm)
w   = P.w;            % w     : spacing of the 2 parallel forearm rods (m, 47 mm)
phi = deg2rad(P.phi); % phi(i): direction of leg i around the base, 180 / 300 / 60 deg (rad)

%% Points of each leg (leg frame i = world frame turned phi(i) about z; its x axis points out along the leg)
for i = 1:3
    Rz   = [cos(phi(i)) -sin(phi(i)) 0; sin(phi(i)) cos(phi(i)) 0; 0 0 1];   % Rz : leg frame i -> world
    v{i} = Rz*[0; 1; 0];                                       % v_i : leg tangent = shoulder axis = rod-end bolt axis
    A{i} = Rz*[R(i); T(i); H];                                 % A_i : shoulder pivot (fixed)
    B{i} = Rz*[R(i) + La*cos(q(i)); T(i); H - La*sin(q(i))];   % B_i : elbow, end of the upper arm (moves with q_i)
    C{i} = p + Rz*[r; 0; 0];                                   % C_i : where the forearm meets the hand (moves with p)
end

%% Equation: forearm length (loop closure)  |C_i - B_i| = Lb
for i = 1:3
    eq(i) = sum((C{i} - B{i}).^2) == Lb^2; %#ok<SAGROW>      % eq(i) : one equation per leg, unknown q_i
end

%% Hand path p(t): circle in x-y while the height bobs up and down, 2 laps
Tlap = 5;      % Tlap : time for one lap (s)
nLap = 2;      % nLap : number of laps
Az   = 0.020;  % Az   : height amplitude (m)
p0   = deltaFK(P.qMid*[1 1 1], P);   % p0 : centre of the path = hand position with every arm at mid-travel
% Rc : circle radius (m) = 90 % of the largest circle inside the rope end-stops and the rod-end tilt limit
Rc = 0.9*min(arrayfun(@(zz) maxRadius(P, zz, 1, p0(1:2)), p0(3) + linspace(-Az, Az, 9)));
pt = [p0(1) + Rc*cos(2*pi*t/Tlap);   % pt : hand position as a function of time, p(t)
      p0(2) + Rc*sin(2*pi*t/Tlap);
      p0(3) + Az*sin(4*pi*t/Tlap)];

%% Solve eq. for q_i(x,y,z)
% Expanding |C_i - B_i|^2 = Lb^2 gives  a*cos(q) + b*sin(q) + c = 0  (cos^2 + sin^2 = 1 removes the squares).
% Half-angle substitution u = tan(q/2): cos q = (1-u^2)/(1+u^2), sin q = 2u/(1+u^2) turns it into the quadratic
% (c-a)*u^2 + 2*b*u + (c+a) = 0  ->  2 solutions per leg (arm up / arm down); keep the arms-up one (as deltaIK).
syms u cq sq                     % u = tan(q/2); cq, sq = stand-ins for cos(q_i), sin(q_i)
qIK  = sym(zeros(3,1));          % qIK  : shoulder angles as functions of the hand position x, y, z
qRef = deg2rad(deltaIK(p0, P)).';   % qRef : numeric answer at p0, used only to pick the right solution
for i = 1:3
    e = subs(expand(lhs(eq(i)) - rhs(eq(i))), [cos(q(i)) sin(q(i))], [cq sq]);  % e = 0 written in cq, sq
    a = subs(diff(e, cq), [cq sq], [0 0]);                 % a : coefficient of cos(q_i)
    b = subs(diff(e, sq), [cq sq], [0 0]);                 % b : coefficient of sin(q_i)
    c = subs(e, [cq sq], [0 0]) + La^2;                    % c : constant term (La^2*(cq^2 + sq^2) = La^2)
    sol = 2*atan(solve((c - a)*u^2 + 2*b*u + (c + a) == 0, u));   % sol : both solutions q = 2*atan(u)
    val = double(subs(sol, p, p0));                        % val : both solutions at p0 (numbers)
    [~, k] = min(abs(mod(val - qRef(i) + pi, 2*pi) - pi)); % k   : the one closest to the arms-up answer
    qIK(i) = sol(k);
end
vpa(qIK, 4)                      % show q1, q2, q3 (x, y, z) with 4 digits (the exact form has huge fractions)

%% Change q(x,y,z) --> q(t): position, velocity, acceleration of the shoulders
q_t   = subs(qIK, p, pt);        % q_t   : shoulder angles q(t) (rad)
dq_t  = diff(q_t, t);            % dq_t  : shoulder angular velocity dq/dt (rad/s)
ddq_t = diff(q_t, t, 2);         % ddq_t : shoulder angular acceleration d2q/dt2 (rad/s^2)
dp_t  = diff(pt, t);             % dp_t  : hand velocity dp/dt (m/s)
ddp_t = diff(pt, t, 2);          % ddp_t : hand acceleration d2p/dt2 (m/s^2)

%% Check against the numeric solvers (deltaIK, delta_kinematics_inv) at a few times
% matlabFunction turns each symbolic result into a fast numeric function of t: qf(t), dqf(t), ...
qf = matlabFunction(q_t); dqf = matlabFunction(dq_t); ddqf = matlabFunction(ddq_t);
pf = matlabFunction(pt);  dpf = matlabFunction(dp_t); ddpf = matlabFunction(ddp_t);
for tt = linspace(0, nLap*Tlap, 13)                       % tt : check times (s)
    [qn, dqn, ddqn] = delta_kinematics_inv(pf(tt), dpf(tt), ddpf(tt));   % numeric q, dq, ddq
    assert(norm(qf(tt) - deg2rad(deltaIK(pf(tt), P)).') < 1e-9, 'q(t) does not match deltaIK');
    assert(norm(dqf(tt) - dqn) < 1e-8*max(1, norm(dqn)) && norm(ddqf(tt) - ddqn) < 1e-8*max(1, norm(ddqn)), ...
        'dq/ddq do not match delta_kinematics_inv');
end
tv = linspace(0, nLap*Tlap, 201);                                  % tv : time samples for the plots (s)
qs = rad2deg(cell2mat(arrayfun(qf, tv, 'UniformOutput', false))); % qs : q at every sample (deg), 3 x 201
ps = cell2mat(arrayfun(pf, tv, 'UniformOutput', false));          % ps : hand position at every sample (m)
assert(all(qs(:) > P.qMin & qs(:) < P.qMax), 'path leaves the rope limits');        % rope end-stops
assert(all(rodEndTilt(ps, P) < P.tiltMax, 'all'), 'path tilts a rod end past its limit');  % rod-end tilt
fprintf('circle radius %.1f mm, height +-%.0f mm; q in [%.1f %.1f] deg, max rod-end tilt %.1f deg\n', ...
    1e3*Rc, 1e3*Az, min(qs(:)), max(qs(:)), max(rodEndTilt(ps, P), [], 'all'));

%% Plot q, dq, ddq vs time (numeric values of the symbolic results: fplot on these long expressions is very slow)
figure('Name', 'Shoulder kinematics')
lab = {'q (deg)', 'dq/dt (deg/s)', 'd^2q/dt^2 (deg/s^2)'}; fn = {qf, dqf, ddqf};
for k = 1:3
    subplot(3, 1, k)
    plot(tv, rad2deg(cell2mat(arrayfun(fn{k}, tv, 'UniformOutput', false))))
    grid on; ylabel(lab{k}); if k == 1, legend('q_1 (leg 1)', 'q_2 (leg 2)', 'q_3 (leg 3)'); end
end
xlabel('t (s)')

%% Change B(q) --> B(t), C(p) --> C(t), evaluated once at every animation frame (fanimator then only draws)
fps = 20;                        % fps : animation frames per second
tf  = 0:1/fps:nLap*Tlap;         % tf  : frame times (s)
for i = 1:3
    Bf = matlabFunction(subs(B{i}, q(i), q_t(i)), 'Vars', t);     % Bf(t) : elbow position B_i(t)
    Cf = matlabFunction(subs(C{i}, p, pt), 'Vars', t);            % Cf(t) : hand attachment C_i(t)
    Bn{i} = cell2mat(arrayfun(Bf, tf, 'UniformOutput', false));   % Bn{i} : elbow at every frame, 3 x frames
    Cn{i} = cell2mat(arrayfun(Cf, tf, 'UniformOutput', false));   % Cn{i} : hand attachment at every frame
end
fr = @(tt) round(tt*fps) + 1;    % fr(tt) : frame number of time tt
% Drawing functions of time: one per colour, all 3 legs in one line (fanimator needs one object per function);
% gap = NaN column splits one line into separate segments
gap = NaN(3,1);
H_arm     = @(tt) line3([A{1}, Bn{1}(:,fr(tt)), gap, A{2}, Bn{2}(:,fr(tt)), gap, A{3}, Bn{3}(:,fr(tt))], '-r');  % upper arms A->B
rodPair   = @(i, k) [Bn{i}(:,k) + w/2*v{i}, Cn{i}(:,k) + w/2*v{i}, gap, Bn{i}(:,k) - w/2*v{i}, Cn{i}(:,k) - w/2*v{i}, gap];  % 2 rods of leg i, w apart
H_forearm = @(tt) line3([rodPair(1, fr(tt)), rodPair(2, fr(tt)), rodPair(3, fr(tt))], '-g');                    % forearms B->C
H_hand    = @(tt) line3([Cn{1}(:,fr(tt)), Cn{2}(:,fr(tt)), Cn{3}(:,fr(tt)), Cn{1}(:,fr(tt))], '-b');              % hand triangle

figure('Name', 'Delta haptic animation')
hold on
Ab = [A{:} A{1}];
plot3(Ab(1,:), Ab(2,:), Ab(3,:), 'b^-')                      % blue triangles: shoulder pivots A_i (base)
for i = 1:3, plot3(Bn{i}(1,:), Bn{i}(2,:), Bn{i}(3,:)), end  % thin arcs: elbow paths B_i(t)
opts = {'AnimationRange', [0 nLap*Tlap], 'FrameRate', fps};
fanimator(H_arm, opts{:}); fanimator(H_forearm, opts{:}); fanimator(H_hand, opts{:});
plot3(ps(1,:), ps(2,:), ps(3,:), '--m')                      % magenta dashes: hand path p(t)
hold off
axis equal; grid on; view(35, 20); xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)')
axis([-0.15 0.15 -0.15 0.15 H - 0.03 max(ps(3,:)) + 0.03])   % hand works above the shoulder pivots (arms up)
title('red: upper arms   green: forearm rods   blue: hand   magenta: hand path')

playAnimation

function h = line3(M, style)   % plot the columns of a 3xN point matrix as one 3D line
    h = plot3(M(1,:), M(2,:), M(3,:), style, 'LineWidth', 2);
end
