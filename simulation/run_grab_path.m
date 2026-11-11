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
