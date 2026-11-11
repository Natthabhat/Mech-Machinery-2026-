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
