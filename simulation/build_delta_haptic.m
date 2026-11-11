%% build_delta_haptic.m  -  Simscape Multibody model of the 3-DOF capstan delta haptic device
% Run from this folder:  build_delta_haptic      -> creates/opens delta_haptic.slx
% Then:  out = sim('delta_haptic');  (Mechanics Explorer shows the 3D model)
%
% Kinematics: each leg = shoulder Revolute (capstan-driven upper arm) + equivalent forearm rod
% with a Universal joint at each end (outer axes parallel to the shoulder axis). This is
% kinematically identical to the rod-end parallelogram (effector stays level, 3 translational
% DOF) and avoids the redundant constraints a 4-bar loop per leg would add.
%
% Conventions: World Z up, base at Z=0. Leg i sits at azimuth phi_i. Joint angle q_i = 0 when
% the upper arm points radially outward and horizontal; q > 0 swings the arm DOWN.

%% ---- Parameters (SI units). MEASURE the ones flagged on the real device. ----
P.phi  = [180 300 60];   % deg, leg azimuths, matching the pads in the base
% Bracket placement from the screw holes (base hole centres in mm, base origin, per leg [180 300 60 deg]).
% Front: bracket round Ø3.1 pair at x=±30.6, z=+1 mm  <-> base Ø3.1 pair, 61.2 mm apart -> fixes the bracket exactly.
% Rear:  bracket Ø4 slot at x=±16.6, z=-55..-42 mm    <-> base Ø4.9 pair, 33.2 mm apart -> must fall inside the slot.
P.baseFile   = 'Base V4.step';
P.holesFront = {[-111.54 18.72; -50.34 18.72], [8.49 -51.61; 39.09 -104.61], [39.78 32.71; 70.38 85.72]};
P.holesRear  = {[-97.55 70.72; -64.35 70.72], [-29.54 -89.74; -12.94 -118.49], [91.82 18.85; 108.42 47.60]};
P.zHole = 0.001;                           % bracket front hole row along its axis (z_b)
P.slot  = [-0.055 -0.042];                 % bracket rear slot extent along z_b
for i = 1:3
    u = [cosd(P.phi(i)) sind(P.phi(i))]; v = [-u(2) u(1)];
    F = P.holesFront{i}; Rr = P.holesRear{i};
    assert(abs(abs(diff(F)*u') - 61.2) < 0.05 && abs(diff(F)*v') < 0.05, 'leg %d: front holes are not the 61.2 mm bracket pair', i);
    assert(abs(abs(diff(Rr)*u') - 33.2) < 0.05 && abs(diff(Rr)*v') < 0.05, 'leg %d: rear holes are not the 33.2 mm slot pair', i);
    P.holeRad(i) = mean(F)*1e-3*u';        % radial -> pivot axis distance (bracket x=0 is the pivot/motor axis)
    P.holeTan(i) = mean(F)*1e-3*v';        % along the leg's tangent (= pivot axis)
    zRear = P.zHole + (mean(Rr) - mean(F))*1e-3*v';   % where the rear base hole lands along the bracket axis
    assert(zRear > P.slot(1) && zRear < P.slot(2), 'leg %d: rear base hole at z=%.1f mm misses the bracket slot', i, 1e3*zRear);
    P.zRear(i) = zRear;                    % rear screw position in the slot (bracket z_b)
    assert(abs((mean(Rr) - mean(F))*u') < 0.05, 'leg %d: rear holes are off the bracket centre line', i);
end
P.R    = P.holeRad;      % per-leg base centre -> shoulder pivot axis (base layout is not exactly 3-fold symmetric)
P.H    = 0.010+0.0241+0.090; % pivot height: base top 10 mm + bracket foot->motor axis 24.1 + motor axis->pivot 90 ('Motor Bracket v6')
P.La   = 0.0721;         % upper arm, pivot -> rod-end crossbar centre (from 'connector v6.STEP' holes; verify)
P.Lb   = 0.170;          % forearm, M6 rod-end eye centre -> eye centre (measured)
P.tiltMax = 13;          % deg, rod-end misalignment limit (typical M6 rod end; check your datasheet) -> see rodEndTilt
P.r    = 0.034;          % effector centre -> rod-end bolt line ('hand V.3.step', sides at 180/60/300 deg)
P.handleTop = 0.027;     % handle-sphere centre above the rod-end bolt plane (2 + 25 mm, hand V.3)
P.ropeD   = 1.5;         % mm, drive rope diameter (visual)
P.slotPos = 8;           % mm, rope-clamp screw position along the drum's tension slot, from its outer end (0..48)
P.N    = 76/9;           % capstan ratio = big-drum cable radius / small-drum groove radius (from STEPs; add cable dia/2 to both if needed)
P.Jrot = 54e-7;          % kg*m^2, 42BYGH40 rotor inertia 54 g*cm^2 (typical datasheet value)
% Capstan travel: rope runs on the 76 mm arc between tie holes at y=+-50 mm -> +-asind(50/76) = +-41 deg of drum turn.
% Rope is centred when the arc centre (drum -X) faces the small drum (90 mm below the pivot).
P.armAng = atan2d(51.96,50);      % deg, arm direction in the connector/drum part frame (boss at (50,-51.96), Z flipped)
P.qMid   = P.armAng - 90;         % deg, shoulder angle with rope centred (-43.9 = arm 44 deg above horizontal)
P.qMin   = P.qMid - asind(50/76); % deg, rope end-stops (q<0 = arm up)
P.qMax   = P.qMid + asind(50/76);
P.kStop  = 2000;                  % N*m/rad, end-stop stiffness: rope pulled tight at its anchor (stiff) - tune
P.cStop  = 2;                     % N*m*s/rad, end-stop damping - tune
P.zSmall = 0.0113;       % small-drum origin above the motor face: plate 9 mm + drum flange 2.3 mm ('Small Drum v5' z=-2.3)
% arm/drum plane offset from the motor face, set by the holes so the mean leg plane passes through the base centre
P.zArm = -mean(P.holeTan) + P.zHole;
P.t    = P.holeTan - P.zHole + P.zArm;   % per-leg leg-plane offset along the tangent (|t| < 0.5 mm)
P.b    = 2e-3;           % N*m*s/rad, shoulder viscous damping (cable + bearings) - tune
P.rho  = 1270;           % kg/m^3, PETG 1.27 g/cm^3, all printed parts; treats them as solid (scale down for partial infill)
P.w    = 0.047;          % parallelogram width: rod-end eye spacing = 38 mm crossbar/boss + 2 x 4.5 mm half ball width (both ends)
P.tubeStart = 0.035;     % Link V.2 tube starts 35 mm from each eye centre: (Lb - 100 mm tube)/2; 16 mm of M6 thread inside
P.rhoSteel  = 7850;      % kg/m^3, screws and bearing (not weighed)
% weighed masses (info.txt), kg; mass spread uniformly over each part's CAD shape. Hand uses P.rho (not weighed).
P.m = struct('bigDrum',0.0313, 'connector',0.0091, 'smallDrum',0.0042, 'bracket',0.0322, ...
             'rodEnd',0.020, 'tube',0.0068, 'motor',0.330, 'base',0.250);
P.m.rodHousing = P.m.rodEnd - 694.8e-9*P.rhoSteel;   % weighed rod end minus its steel ball (694.8 mm^3, modelled separately)
P.q0   = -40;            % deg, assembly guess (arms up)
P.dir  = fileparts(mfilename('fullpath'));                        % this folder: models (.slx) are saved here
P.cad  = fullfile(P.dir, '..', 'hardware', 'mechanical', 'cad');  % STEP files in the GitHub repo layout
if ~isfolder(P.cad), P.cad = P.dir; end                          % or all files in one folder
assignin('base','P',P);

mdl = 'delta_haptic';
if bdIsLoaded(mdl), close_system(mdl,0); end
if exist(fullfile(P.dir,[mdl '.slx']),'file'), delete(fullfile(P.dir,[mdl '.slx'])); end
new_system(mdl); load_system('sm_lib'); load_system('nesl_utility');
assignin(get_param(mdl,'ModelWorkspace'),'P',P);   % saved inside the .slx, so it opens/simulates without this script
set_param(mdl,'StopTime','2','SolverType','Variable-step','Solver','ode23t','RelTol','1e-4');

%% ---- Environment ----
W   = blk('sm_lib/Frames and Transforms/World Frame','World',[20 200]);
MC  = blk('sm_lib/Utilities/Mechanism Configuration','MechConfig',[20 300],'GravityVector','[0 0 -9.81]');
SC  = blk('nesl_utility/Solver Configuration','Solver',[20 400]);
con(W,'R1',MC,'R1'); con(W,'R1',SC,'R1');
BASE = blk('sm_lib/Body Elements/File Solid','Base',[120 200],'ExtGeomFileName',fullfile(P.cad,P.baseFile), ...
    'UnitType','Custom','ExtGeomFileUnits','mm','InertiaType','CalculateFromGeometry','BasedOnType','Density','Density','P.rho','GraphicDiffuseColor','[0.4 0.85 0.75]');
con(W,'R1',BASE,'R1');

% Motor torque command [3x1] N*m at the motor shafts (default zero = free motion under gravity)
TM  = blk('simulink/Sources/Constant','MotorTorque',[20 520],'Value','[0;0;0]');
DM  = blk('simulink/Signal Routing/Demux','Demux',[120 520],'Outputs','3');
add_line(mdl,'MotorTorque/1','Demux/1');

% Effector: EFF's base port = centre of the rod-end bolt plane (measured point); hand part origin is 2 mm above it
EFF = blk('sm_lib/Frames and Transforms/Rigid Transform','HandOffset',[1300 700],'RotationMethod','None', ...
    'TranslationMethod','Cartesian','TranslationCartesianOffset','[0 0 0.002]');
HND = blk('sm_lib/Body Elements/File Solid','HandPart',[1400 700],'ExtGeomFileName',fullfile(P.cad,'hand V.3.step'), ...
    'UnitType','Custom','ExtGeomFileUnits','mm','InertiaType','CalculateFromGeometry','BasedOnType','Density', ...
    'Density','P.rho','GraphicDiffuseColor','[0.4 0.85 0.75]');
con(EFF,'R1',HND,'R1');
TS  = blk('sm_lib/Frames and Transforms/Transform Sensor','EffectorPos',[1300 850],'SenseXYZ','on');
PSx = blk('nesl_utility/PS-Simulink Converter','xyz_PS',[1450 850]);
TWx = blk('simulink/Sinks/To Workspace','xyz',[1550 850],'VariableName','xyz','SaveFormat','Timeseries');
con(W,'R1',TS,'L1'); con(EFF,'L1',TS,'R1'); con(TS,'R2',PSx,'L1');
add_line(mdl,'xyz_PS/1','xyz/1');

%% ---- Legs ----
for i = 1:3
    y = 50 + (i-1)*200; s = num2str(i); ph = sprintf('P.phi(%d)',i);
    % base -> shoulder base frame: rotate to leg azimuth, move out by R, joint Z -> tangent (Y_leg)
    Taz = blk('sm_lib/Frames and Transforms/Rigid Transform',['Az' s],[250 y],'RotationMethod','StandardAxis', ...
        'RotationStandardAxis','+Z','RotationAngle',ph,'RotationAngleUnits','deg', ...
        'TranslationMethod','None');
    Tsh = blk('sm_lib/Frames and Transforms/Rigid Transform',['ShoulderFrame' s],[350 y],'RotationMethod','RotationMatrix', ...
        'RotationMatrix','[1 0 0; 0 0 1; 0 -1 0]','TranslationMethod','Cartesian','TranslationCartesianOffset',sprintf('[P.R(%d) P.t(%d) P.H]',i,i));
    con(W,'R1',Taz,'L1'); con(Taz,'R1',Tsh,'L1');
    % static visuals: bracket frame = shoulder frame shifted to the motor axis (90 mm below pivot)
    Tbr = blk('sm_lib/Frames and Transforms/Rigid Transform',['BracketFrame' s],[350 y-40],'RotationMethod','None', ...
        'TranslationMethod','Cartesian','TranslationCartesianOffset','[0 0.090 -P.zArm]');
    BR  = blk('sm_lib/Body Elements/File Solid',['Bracket' s],[470 y-40],'ExtGeomFileName',fullfile(P.cad,'Motor Bracket v6.STEP'), ...
        'UnitType','Custom','ExtGeomFileUnits','mm','InertiaType','CalculateFromGeometry','BasedOnType','Density','Density','P.rho','GraphicDiffuseColor','[0.4 0.85 0.75]');
    MO  = blk('sm_lib/Body Elements/File Solid',['Motor' s],[470 y-80],'ExtGeomFileName',fullfile(P.cad,'Nema17 42BYGH40.step'), ...
        'UnitType','Custom','ExtGeomFileUnits','mm','InertiaType','CalculateFromGeometry','BasedOnType','Density', ...
        'Density','P.rho','GraphicDiffuseColor','[0.15 0.15 0.15]');   % exported from Fusion 'Nema17'; static, mass irrelevant
    % motor file: shaft axis at (20,20) mm, mounting face at z=40 mm -> put face on bracket plate (z=0), shaft along +z
    Tmo = blk('sm_lib/Frames and Transforms/Rigid Transform',['MotorFrame' s],[350 y-80],'RotationMethod','None', ...
        'TranslationMethod','Cartesian','TranslationCartesianOffset','[-0.020 -0.020 -0.040]');
    con(Tsh,'R1',Tbr,'L1'); con(Tbr,'R1',BR,'R1'); con(Tbr,'R1',Tmo,'L1'); con(Tmo,'R1',MO,'R1');
    % bracket hardware (bracket frame, mm): 4x M2.5x6 motor screws, heads in the Ø5.5 counterbores (seat z=4.5),
    % shank toward the motor; 2x M3x6 into the base, heads on the foot flange (y=21.1), shank down (+y) into the base
    footR = [1 0 0; 0 0 -1; 0 1 0];          % screw file Z (head up) -> bracket -Y (up)
    bracketHW{i} = {};
    for c = [-1 -1; -1 1; 1 -1; 1 1]'
        bracketHW{i}{end+1} = mount(Tbr, sprintf('MotorScrew%s_%d%d', s, c(1)>0, c(2)>0), 'M2.5x6 ISO4762.step', [15.5*c' 4.5], eye(3));
    end
    for xs = [-1 1]
        bracketHW{i}{end+1} = mount(Tbr, sprintf('BaseScrew%s_%d', s, xs>0), 'M3x6 ISO4762.step', [30.6*xs 21.1 1e3*P.zHole], footR);
        % behind the motor: M3x6 in the Ø4 slot (x=±16.6), at the base hole's position along the slot
        bracketHW{i}{end+1} = mount(Tbr, sprintf('BaseScrewRear%s_%d', s, xs>0), 'M3x6 ISO4762.step', [16.6*xs 21.1 1e3*P.zRear(i)], footR);
    end
    bracketHW{i} = [bracketHW{i}{:}];

    J = blk('sm_lib/Joints/Revolute Joint',['Shoulder' s],[470 y],'TorqueActuationMode','NoTorque', ...
        'SensePosition','on','DampingCoefficient','P.b','DampingCoefficientUnits','N*m*s/rad', ...   % default unit is per deg/s
        'PositionTargetSpecify','on', ...
        'PositionTargetValue','P.q0','PositionTargetValueUnits','deg','PositionTargetPriority','Low', ...
        'LowerLimitSpecify','on','LowerLimitBound','P.qMin','LowerLimitBoundUnits','deg', ...
        'LowerLimitStiffness','P.kStop','LowerLimitStiffnessUnits','N*m/rad','LowerLimitDamping','P.cStop', ...
        'LowerLimitDampingUnits','N*m*s/rad','LowerLimitTransitionRegionWidth','0.5','LowerLimitTransitionRegionWidthUnits','deg', ...
        'UpperLimitSpecify','on','UpperLimitBound','P.qMax','UpperLimitBoundUnits','deg', ...
        'UpperLimitStiffness','P.kStop','UpperLimitStiffnessUnits','N*m/rad','UpperLimitDamping','P.cStop', ...
        'UpperLimitDampingUnits','N*m*s/rad','UpperLimitTransitionRegionWidth','0.5','UpperLimitTransitionRegionWidthUnits','deg');
    con(Tsh,'R1',J,'L1');
    PSq = blk('nesl_utility/PS-Simulink Converter',['q_PS' s],[600 y+90],'Unit','deg');
    TWq = blk('simulink/Sinks/To Workspace',['q' s],[700 y+90],'VariableName',['q' s],'SaveFormat','Timeseries');
    con(J,'R2',PSq,'L1'); add_line(mdl,['q_PS' s '/1'],['q' s '/1']);

    % motor shaft + small drum: revolute on the motor axis (bracket frame), driven by the motor torque
    JM = blk('sm_lib/Joints/Revolute Joint',['MotorShaft' s],[470 y-120],'TorqueActuationMode','InputTorque', ...
        'SensePosition','on');
    con(Tbr,'R1',JM,'L1');
    SP = blk('nesl_utility/Simulink-PS Converter',['tau_PS' s],[350 y+90],'Unit','N*m');
    add_line(mdl,['Demux/' s],['tau_PS' s '/1']); con(SP,'R1',JM,'L2');
    PSm = blk('nesl_utility/PS-Simulink Converter',['qm_PS' s],[600 y+130],'Unit','deg');
    TWm = blk('simulink/Sinks/To Workspace',['qm' s],[700 y+130],'VariableName',['qm' s],'SaveFormat','Timeseries');
    con(JM,'R2',PSm,'L1'); add_line(mdl,['qm_PS' s '/1'],['qm' s '/1']);
    Jr = blk('sm_lib/Body Elements/Inertia',['Rotor' s],[600 y-160],'InertiaType','Custom', ...
        'Mass','1e-6','MomentsOfInertia','P.Jrot*[0.5 0.5 1]','GraphicType','None');   % only Izz acts (shaft axis)
    Tsd = blk('sm_lib/Frames and Transforms/Rigid Transform',['SmallDrumFrame' s],[600 y-120],'RotationMethod','None', ...
        'TranslationMethod','Cartesian','TranslationCartesianOffset','[0 0 P.zSmall]');
    SD  = blk('sm_lib/Body Elements/File Solid',['SmallDrum' s],[720 y-120],'ExtGeomFileName',fullfile(P.cad,'Small Drum v5.STEP'), ...
        'UnitType','Custom','ExtGeomFileUnits','mm','InertiaType','CalculateFromGeometry','BasedOnType','Density', ...
        'Density','P.rho','GraphicDiffuseColor','[0.1 0.2 0.8]');
    % rope wraps on the small drum (visual + negligible mass): tube r 9..10.5 mm over 10 mm of the groove
    RS  = blk('sm_lib/Body Elements/Revolved Solid',['RopeSmall' s],[840 y-120],'RevolutionCrossSection', ...
        '[9 13; 10.5 13; 10.5 23; 9 23]','RevolutionCrossSectionUnits','mm','RevolutionExtent','Full', ...
        'InertiaType','PointMass','Mass','1e-4','GraphicDiffuseColor','[0.9 0.2 0.2]');
    con(JM,'R1',Jr,'R1'); con(JM,'R1',Tsd,'L1'); con(Tsd,'R1',SD,'R1'); con(Tsd,'R1',RS,'R1');
    % rope = no-slip capstan: kinematically an external gear pair, ratio N, real 90 mm centre distance
    GC  = blk('sm_lib/Gears and Couplings/Gears/Common Gear Constraint',['Rope' s],[600 y-80],'GearType','External', ...
        'SpecificationType','CenterDistanceRatio','CenterDistance','0.090','CenterDistanceUnits','m','GearRatio','P.N');
    Tgc = blk('sm_lib/Frames and Transforms/Rigid Transform',['RopePlane' s],[470 y-80],'RotationMethod','None', ...
        'TranslationMethod','Cartesian','TranslationCartesianOffset','[0 0 P.zArm]');   % gear frames must be coplanar
    con(JM,'R1',Tgc,'L1'); con(Tgc,'R1',GC,'L1'); con(J,'R1',GC,'R1');
    % connector + drum share one frame (common hub, boss at (20,0)); flip Z, then turn the arm boss onto +X
    Rc = blk('sm_lib/Frames and Transforms/Rigid Transform',['ConnAlign' s],[600 y],'RotationMethod','RotationMatrix', ...
        'RotationMatrix','[cosd(P.armAng) sind(P.armAng) 0; -sind(P.armAng) cosd(P.armAng) 0; 0 0 1]*diag([1 -1 -1])', ...
        'TranslationMethod','None');
    FC = blk('sm_lib/Body Elements/File Solid',['Arm' s],[720 y],'ExtGeomFileName',fullfile(P.cad,'connector v6.STEP'), ...
        'UnitType','Custom','ExtGeomFileUnits','mm','InertiaType','CalculateFromGeometry','BasedOnType','Density', ...
        'Density','P.rho','GraphicDiffuseColor','[0.1 0.2 0.8]');
    FD = blk('sm_lib/Body Elements/File Solid',['BigDrum' s],[720 y-40],'ExtGeomFileName',fullfile(P.cad,'Big Drum V6.STEP'), ...
        'UnitType','Custom','ExtGeomFileUnits','mm','InertiaType','CalculateFromGeometry','BasedOnType','Density', ...
        'Density','P.rho','GraphicDiffuseColor','[0.1 0.2 0.8]');
    con(J,'R1',Rc,'L1'); con(Rc,'R1',FC,'R1'); con(Rc,'R1',FD,'R1');
    % rope lying on the big-drum arc between the tie holes (drum part frame: arc centre on -X, groove centre z=3.1 mm)
    Tra = blk('sm_lib/Frames and Transforms/Rigid Transform',['RopeArcFrame' s],[720 y+40],'RotationMethod','StandardAxis', ...
        'RotationStandardAxis','+Z','RotationAngle','180','RotationAngleUnits','deg', ...   % a custom revolve is centred on +X
        'TranslationMethod','Cartesian','TranslationCartesianOffset','[0 0 0.0031]');
    th = 'linspace(0,2*pi,13)''';
    RA  = blk('sm_lib/Body Elements/Revolved Solid',['RopeArc' s],[840 y+40],'RevolutionCrossSection', ...
        ['[76.75+0.75*cos(' th ') 0.75*sin(' th ')]'],'RevolutionCrossSectionUnits','mm','RevolutionExtent','Custom', ...
        'RevolutionAngle','2*asind(50/76)','RevolutionAngleUnits','deg', ...
        'InertiaType','PointMass','Mass','1e-4','GraphicDiffuseColor','[0.9 0.2 0.2]');
    con(Rc,'R1',Tra,'L1'); con(Tra,'R1',RA,'R1');
    % hub: two 606ZZ, one in each Ø16.9 pocket (drum file z -5.9..0.1 and 6.1..12.1), flush with the hub faces
    silver = [0.75 0.75 0.78];
    armHW{i} = [mount(Rc, ['BearingIn' s],  '606ZZ.step', [0 0 -2.9], eye(3), silver), ...
                mount(Rc, ['BearingOut' s], '606ZZ.step', [0 0  9.1], eye(3), silver)];
    % drum stub (20,0): M3x6 head in the Ø6x1 counterbore (seat z=-1.4), through Ø3 into the connector boss
    armHW{i} = [armHW{i}, mount(Rc, ['DrumScrew' s], 'M3x6 ISO4762.step', [20 0 -1.4], diag([1 -1 -1]))];
    % rope anchors at both arc ends (drum part frame, mm): groove -> Ø4 hole along X (y=±50.15, z=3.1) -> out on the
    % side wall's outer face -> wrapped under an M3x10 in the wall's Ø4 slot, nut on the inside face (slide to tension)
    for sg = [-1 1]
        e  = sprintf('%s%c', s, 'ab'*[sg<0; sg>0]);
        n  = [0.585 0.811*sg 0];  u = [0.811 -0.585*sg 0];   % wall normal (outward), slot direction (toward the hub)
        p0 = [-45.91 42.33*sg 3.1];                         % slot outer-end centre on the wall's inside face
        c5 = p0 + 5*n + P.slotPos*u;                        % screw axis on the outside face (wall 5 mm thick)
        hole = [-48.22 50.15*sg 3.1];                       % Ø4 hole exit on the outside face
        rr = P.ropeD/2;
        armHW{i} = [armHW{i}, ...
            ropeSeg(Rc, ['RopeHole' e], [-58.10 50.15*sg 3.1], hole, rr), ...
            ropeSeg(Rc, ['RopeFace' e], hole + rr*n, c5 + rr*n - (1.5 + rr)*u, rr), ...
            ropeRing(Rc, ['RopeWrap' e], c5 + rr*n, n, 1.5 + rr, rr), ...
            mount(Rc, ['RopeScrew' e], 'M3x10 ISO4762.step', c5 + P.ropeD*n, axesZ(n)), ...      % head clamps the rope
            mount(Rc, ['RopeNut' e], 'M3 nut ISO4032.step', p0 + P.slotPos*u, axesZ(-n))];       % nut on the inside face
    end

    % elbow: arm end, U-joint frame with X = pivot axis (parallel to shoulder), Y = radial, Z = down
    Tel = blk('sm_lib/Frames and Transforms/Rigid Transform',['Elbow' s],[850 y],'RotationMethod','RotationMatrix', ...
        'RotationMatrix','[0 1 0; 0 0 1; 1 0 0]','TranslationMethod','Cartesian','TranslationCartesianOffset','[P.La 0 0]');
    con(J,'R1',Tel,'L1');
    % effector side: same frame recipe as the shoulder side, at radius r (X=tangent, Y=radial, Z=down at q=0)
    Ea = blk('sm_lib/Frames and Transforms/Rigid Transform',['EffAz' s],[1150 700+30*i],'RotationMethod','StandardAxis', ...
        'RotationStandardAxis','+Z','RotationAngle',ph,'RotationAngleUnits','deg','TranslationMethod','None');
    Ep = blk('sm_lib/Frames and Transforms/Rigid Transform',['EffPivot' s],[1250 y],'RotationMethod','RotationMatrix', ...
        'RotationMatrix','[0 1 0; 1 0 0; 0 0 -1]','TranslationMethod','Cartesian','TranslationCartesianOffset','[P.r 0 0]');
    con(EFF,'L1',Ea,'L1'); con(Ea,'R1',Ep,'L1');

    % forearm = real parallelogram: 2 rods, each = rod end + Link V.2 tube + rod end, spherical joint at each eye.
    % Rod frame: origin at the elbow eye centre, rod along -Z (up at q=0), eye bores along X.
    for side = [-1 1]
        t = [s char('a'+(side>0))]; sg = num2str(side);
        eyeA = blk('sm_lib/Frames and Transforms/Rigid Transform',['ElbowEye' t],[950 y+20*side],'RotationMethod','None', ...
            'TranslationMethod','Cartesian','TranslationCartesianOffset',['[' sg '*P.w/2 0 0]']);
        eyeB = blk('sm_lib/Frames and Transforms/Rigid Transform',['HandEye' t],[1350 y+20*side],'RotationMethod','None', ...
            'TranslationMethod','Cartesian','TranslationCartesianOffset',['[' sg '*P.w/2 0 0]']);
        % elbow end: universal joint, 1st axis = bolt axis (X, fixed in the arm), so the housing cannot spin on the rod;
        % hand end: ball joint. Same kinematics as ball joints at both ends, without the free rod spin.
        SA = blk('sm_lib/Joints/Universal Joint',['RodEndA' t],[1000 y+20*side]);
        SB = blk('sm_lib/Joints/Spherical Joint',['RodEndB' t],[1400 y+20*side]);
        con(Tel,'R1',eyeA,'L1'); con(eyeA,'R1',SA,'L1'); con(Ep,'R1',eyeB,'L1'); con(eyeB,'R1',SB,'L1');
        % rod-end balls are clamped by the M6 screws: fixed to the arm / hand, ball axis (file Z) along the bolt (X)
        Rball = [0 0 1; 0 1 0; -1 0 0]; silver = [0.75 0.75 0.78];
        mount(eyeA, ['RodBallA' t], 'M6 rod end ball.step', [0 0 0], Rball, silver);
        mount(eyeB, ['RodBallB' t], 'M6 rod end ball.step', [0 0 0], Rball, silver);
        % rod-end file: eye axis Z, stud +Y (51 mm). Elbow end stud -> -Z_rod; hand end stud -> +Z_rod
        RA = blk('sm_lib/Body Elements/File Solid',['RodEndPartA' t],[1100 y+20*side],'ExtGeomFileName',fullfile(P.cad,'M6 rod end housing L51.step'), ...
            'UnitType','Custom','ExtGeomFileUnits','mm','InertiaType','CalculateFromGeometry','BasedOnType','Density', ...
            'Density','P.rhoSteel','GraphicDiffuseColor','[0.75 0.75 0.78]');
        Tb = blk('sm_lib/Frames and Transforms/Rigid Transform',['RodFar' t],[1200 y+20*side],'RotationMethod','RotationMatrix', ...
            'RotationMatrix','[0 0 1; 1 0 0; 0 1 0]','TranslationMethod','Cartesian','TranslationCartesianOffset','[0 0 -P.Lb]');
        RB = blk('sm_lib/Body Elements/File Solid',['RodEndPartB' t],[1300 y+20*side],'ExtGeomFileName',fullfile(P.cad,'M6 rod end housing L51.step'), ...
            'UnitType','Custom','ExtGeomFileUnits','mm','InertiaType','CalculateFromGeometry','BasedOnType','Density', ...
            'Density','P.rhoSteel','GraphicDiffuseColor','[0.75 0.75 0.78]');
        Tr = blk('sm_lib/Frames and Transforms/Rigid Transform',['RodNear' t],[1050 y+20*side],'RotationMethod','RotationMatrix', ...
            'RotationMatrix','[0 0 1; -1 0 0; 0 -1 0]','TranslationMethod','None');
        Ttu = blk('sm_lib/Frames and Transforms/Rigid Transform',['TubeFrame' t],[1150 y+20*side],'RotationMethod','None', ...
            'TranslationMethod','Cartesian','TranslationCartesianOffset','[0 P.tubeStart 0]');   % along the stud axis (+Y of file frame)
        TU = blk('sm_lib/Body Elements/File Solid',['Tube' t],[1250 y+20*side],'ExtGeomFileName',fullfile(P.cad,'Link V.2.step'), ...
            'UnitType','Custom','ExtGeomFileUnits','mm','InertiaType','CalculateFromGeometry','BasedOnType','Density', ...
            'Density','P.rho','GraphicDiffuseColor','[0.4 0.85 0.75]');
        con(SA,'R1',Tr,'L1'); con(Tr,'R1',RA,'R1'); con(Tr,'R1',Ttu,'L1'); con(Ttu,'R1',TU,'R1');
        % hand-end rod end: rod frame -> far eye (Z offset is in the rod frame, so branch from the spherical follower)
        con(SA,'R1',Tb,'L1'); con(Tb,'R1',RB,'R1');
        Tf = blk('sm_lib/Frames and Transforms/Rigid Transform',['RodFarEye' t],[1300 y+20*side+10],'RotationMethod','None', ...
            'TranslationMethod','Cartesian','TranslationCartesianOffset','[0 0 -P.Lb]');
        con(SA,'R1',Tf,'L1'); con(Tf,'R1',SB,'R1');
        % M6x12 screws through each eye: head seated on the ball face (4.5 mm out), shank pointing inward
        rot = sprintf('[0 0 %d; 0 1 0; %d 0 0]', side, -side);
        for e = {{eyeA,'ScrewArm'},{eyeB,'ScrewHand'}}
            Ts = blk('sm_lib/Frames and Transforms/Rigid Transform',[e{1}{2} 'Frame' t],[1000 y+20*side+40],'RotationMethod','RotationMatrix', ...
                'RotationMatrix',rot,'TranslationMethod','Cartesian','TranslationCartesianOffset',['[' sg '*0.0045 0 0]']);
            SC = blk('sm_lib/Body Elements/File Solid',[e{1}{2} t],[1100 y+20*side+40],'ExtGeomFileName',fullfile(P.cad,'M6x12 ISO4762.step'), ...
                'UnitType','Custom','ExtGeomFileUnits','mm','InertiaType','CalculateFromGeometry','BasedOnType','Density', ...
                'Density','P.rhoSteel','GraphicDiffuseColor','[0.15 0.15 0.15]');
            con(e{1}{1},'R1',Ts,'L1'); con(Ts,'R1',SC,'R1');
        end
    end
end
%% ---- group into nested subsystems for a readable diagram ----
%% ---- weighed masses: switch those File Solids from density to mass ----
massOf = {'Base','base'; 'Bracket\d','bracket'; 'Motor\d','motor'; 'SmallDrum\d','smallDrum'; 'Arm\d','connector'; ...
          'BigDrum\d','bigDrum'; 'RodEndPart[AB]\d[ab]','rodHousing'; 'Tube\d[ab]','tube'};
for k = 1:size(massOf,1)
    h = find_system(mdl, 'SearchDepth', 1, 'RegExp', 'on', 'Name', ['^' massOf{k,1} '$']);
    assert(~isempty(h), 'no block matches %s', massOf{k,1});
    for j = 1:numel(h), set_param(h{j}, 'BasedOnType', 'Mass', 'Mass', ['P.m.' massOf{k,2}], 'MassUnits', 'kg'); end
end

grp('Environment', {'World','MechConfig','Solver','Base'});
grp('Hand', {'HandOffset','HandPart','EffectorPos','xyz_PS','xyz'});
for i = 1:3
    s = num2str(i);
    grp(['Motor & bracket ' s], [strcat({'Az','ShoulderFrame','BracketFrame','Bracket','MotorFrame','Motor'}, s), bracketHW{i}]);
    grp(['Drive ' s], strcat({'MotorShaft','Rotor','SmallDrumFrame','SmallDrum','RopeSmall','RopePlane','Rope', ...
                              'tau_PS','qm_PS','qm'}, s));
    grp(['Upper arm ' s], [strcat({'Shoulder','q_PS','q','ConnAlign','Arm','BigDrum','RopeArcFrame','RopeArc', ...
                                   'Elbow'}, s), armHW{i}]);
    for t = strcat(s, {'a','b'})
        grp(['Forearm ' t{1}], [strcat({'ElbowEye','RodEndA','RodNear','RodEndPartA','TubeFrame','Tube','RodFar', ...
            'RodEndPartB','RodFarEye','RodEndB','HandEye','ScrewArmFrame','ScrewArm','ScrewHandFrame','ScrewHand', ...
            'RodBallA','RodBallB'}, t{1}), strcat({'RodBallA','RodBallB'}, t{1}, 'Frame')]);   % mount() names frames <name>Frame
    end
    grp(['Leg ' s], {['Motor & bracket ' s], ['Drive ' s], ['Upper arm ' s], ['Forearm ' s 'a'], ['Forearm ' s 'b'], ...
                     ['EffAz' s], ['EffPivot' s]});
end
subs = find_system(mdl, 'BlockType', 'SubSystem', 'LinkStatus', 'none', 'Mask', 'off');   % only ours, not library blocks
for k = 1:numel(subs), nameports(subs{k}); end                  % top-down, so inner ports reuse outer names
for k = numel(subs):-1:1, Simulink.BlockDiagram.arrangeSystem(subs{k}); end
Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl, fullfile(P.dir,[mdl '.slx']));
fprintf('Built %s.slx\n', mdl);

%% ---- helpers ----
function R = axesZ(z)
    % right-handed axes with Z along z and X along the drum axis (0,0,1) (z must be perpendicular to it)
    z = z(:)/norm(z); x = [0;0;1]; R = [x, cross(z, x), z];
end
function names = ropeSeg(parent, name, A, B, rr)
    % straight rope piece (red cylinder, radius rr mm) from A to B (mm) in the parent frame
    d = B - A; L = norm(d); z = d(:)/L; a = [1;0;0]; if abs(z'*a) > 0.9, a = [0;1;0]; end
    x = cross(a, z); x = x/norm(x); R = [x, cross(z, x), z];
    T = blk('sm_lib/Frames and Transforms/Rigid Transform', [name 'Frame'], [0 0], 'RotationMethod', 'RotationMatrix', ...
        'RotationMatrix', mat2str(R), 'TranslationMethod', 'Cartesian', 'TranslationCartesianOffset', mat2str((A + B)/2000));
    S = blk('sm_lib/Body Elements/Cylindrical Solid', name, [0 0], 'CylinderRadius', num2str(rr/1000), 'CylinderLength', ...
        num2str(L/1000), 'InertiaType', 'PointMass', 'Mass', '1e-5', 'GraphicDiffuseColor', '[0.9 0.2 0.2]');
    con(parent, 'R1', T, 'L1'); con(T, 'R1', S, 'R1');
    names = {[name 'Frame'], name};
end
function names = ropeRing(parent, name, C, axis, R0, rr)
    % rope loop around a screw: torus of tube radius rr at radius R0 (mm), centred at C, axis along 'axis'
    th = linspace(0, 2*pi, 13)';
    T = blk('sm_lib/Frames and Transforms/Rigid Transform', [name 'Frame'], [0 0], 'RotationMethod', 'RotationMatrix', ...
        'RotationMatrix', mat2str(axesZ(axis)), 'TranslationMethod', 'Cartesian', 'TranslationCartesianOffset', mat2str(C/1000));
    S = blk('sm_lib/Body Elements/Revolved Solid', name, [0 0], 'RevolutionCrossSection', ...
        mat2str([R0 + rr*cos(th), rr*sin(th)], 4), 'RevolutionCrossSectionUnits', 'mm', 'RevolutionExtent', 'Full', ...
        'InertiaType', 'PointMass', 'Mass', '1e-5', 'GraphicDiffuseColor', '[0.9 0.2 0.2]');
    con(parent, 'R1', T, 'L1'); con(T, 'R1', S, 'R1');
    names = {[name 'Frame'], name};
end
function names = mount(parent, name, file, pos_mm, R, color)
    % rigidly attach a steel CAD part (bearing/screw) to a frame: part origin at pos_mm, part axes = columns of R
    if nargin < 6, color = [0.2 0.2 0.22]; end
    P = evalin('base', 'P');
    T = blk('sm_lib/Frames and Transforms/Rigid Transform', [name 'Frame'], [0 0], 'RotationMethod', 'RotationMatrix', ...
        'RotationMatrix', mat2str(R), 'TranslationMethod', 'Cartesian', 'TranslationCartesianOffset', mat2str(pos_mm/1000));
    S = blk('sm_lib/Body Elements/File Solid', name, [0 0], 'ExtGeomFileName', fullfile(P.cad, file), 'UnitType', 'Custom', ...
        'ExtGeomFileUnits', 'mm', 'InertiaType', 'CalculateFromGeometry', 'BasedOnType', 'Density', 'Density', 'P.rhoSteel', ...
        'GraphicDiffuseColor', mat2str(color));
    con(parent, 'R1', T, 'L1'); con(T, 'R1', S, 'R1');
    names = {[name 'Frame'], name};
end
function grp(name, blocks)
    % move top-level blocks into a new subsystem; ports for crossing connections are created automatically
    h = cellfun(@(n) getSimulinkBlockHandle(['delta_haptic/' n]), blocks);
    assert(all(h > 0), 'grp %s: missing block(s) %s', name, strjoin(blocks(h < 0), ', '));
    Simulink.BlockDiagram.createSubsystem(h, 'Name', name);
end
function nameports(sub)
    % rename a subsystem's Inport / Connection Port blocks after the block(s) they connect to outside
    pc = get_param(sub, 'PortConnectivity');
    pm = find_system(sub, 'SearchDepth', 1, 'BlockType', 'PMIOPort');
    side = get_param(pm, 'Side'); num = str2double(get_param(pm, 'Port'));
    used = {};
    for k = 1:numel(pc)
        ty = pc(k).Type;
        if startsWith(ty, {'LConn','RConn'})
            if ty(1) == 'L', sd = 'Left'; else, sd = 'Right'; end
            j = find(strcmp(side, sd));
            [~, o] = sort(num(j)); j = j(o); inner = pm{j(str2double(ty(6:end)))};
            nb = [pc(k).DstBlock(:); pc(k).SrcBlock(:)];
        elseif ~isempty(pc(k).SrcBlock) && pc(k).SrcBlock ~= -1      % signal inport
            inner = find_system(sub, 'SearchDepth', 1, 'BlockType', 'Inport', 'Port', ty);
            inner = inner{1}; nb = pc(k).SrcBlock;
        else
            continue
        end
        nb = nb(nb > 0); n = unique(cellstr(get_param(nb, 'Name')));   % one neighbour -> char, wrap it
        % short role names: shared frames are named by their owner, peers collapse to a plural
        roots = unique(regexprep(n, '\s*\d+[ab]?$', ''));
        if endsWith(sub, '/Environment') || any(ismember(n, {'Environment','World'})), nm = 'World';
        elseif any(strcmp(n, 'Hand')) || any(startsWith(n, 'EffPivot')),       nm = 'Hand';
        elseif any(startsWith(n, 'Upper arm')),                                  nm = 'Arm';
        elseif any(strcmp(n, 'Demux')),                                          nm = 'Motor torque';
        elseif isscalar(roots) && numel(n) > 1,                                  nm = [roots{1} 's'];
        else,                                                                    nm = strjoin(roots, ', ');
        end
        taken = setdiff(get_param(find_system(sub, 'SearchDepth', 1, 'Type', 'block'), 'Name'), {get_param(inner, 'Name')});
        if any(strcmp(taken, nm)), nm = [nm ' frame']; end           % e.g. the World Frame block inside Environment
        base = nm; c = 1;
        while any(strcmp(used, nm)) || any(strcmp(taken, nm)), c = c + 1; nm = sprintf('%s (%d)', base, c); end
        used{end+1} = nm; %#ok<AGROW>
        set_param(inner, 'Name', strrep(nm, '/', '//'));
    end
end
function h = blk(src, name, pos, varargin)
    h = add_block(src, ['delta_haptic/' name], 'Position', [pos pos+[60 40]], varargin{:});
end
function con(a, pa, b, pb)
    % pa/pb: 'L1','R2',... = LConn/RConn index of a Simscape port
    A = get_param(a,'PortHandles'); B = get_param(b,'PortHandles');
    ha = A.([pa(1) 'Conn'])(str2double(pa(2:end))); hb = B.([pb(1) 'Conn'])(str2double(pb(2:end)));
    add_line('delta_haptic', ha, hb, 'autorouting','on');
end
