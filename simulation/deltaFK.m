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
