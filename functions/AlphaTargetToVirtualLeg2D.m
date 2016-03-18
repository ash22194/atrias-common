%% AlphaTargetToVirtualLeg2D    Converts alpha target angle into a virtual leg length/angle.
%
%    Lengths: meters
%    Angles:  radians
%    Time:    seconds
%
%    [l, dl, ddl, phi, dphi, ddphi] = AlphaTargetToVirtualLeg2D(alpha_v, dalpha_v, ddalpha_v, theta, dtheta, ddtheta, d_y_com, l0_virtual)
%    converts virtual alpha target angle (alpha_v) and torso pitch angle (theta) into virtual leg length (l_leg) and 
%    virtual leg angle (phi). Velocities (d-prefix) and accelerations (dd-prefix) are similarly transformed. d_y_com is
%    the distance between the COM and sagittal rotation point. l0_virtual is the rest length of the virtual spring.
function [l_leg, dl_leg, ddl_leg, phi, dphi, ddphi] = AlphaTargetToVirtualLeg2D(alpha_v, dalpha_v, ddalpha_v, theta, dtheta, ddtheta, d_y_com, l0_virtual)

    % short variable names
    l_v = l0_virtual;
    d = d_y_com;
    th = theta;
    dth = dtheta;
    ddth =  ddtheta;
    al = alpha_v;
    dal = dalpha_v;
    ddal = ddalpha_v;
    angle = th - al + pi/2;
    
    % positions
    l_leg = (d^2 + l_v^2 - 2*d*l_v*cos(angle))^0.5;
    phi = pi/2 - th + atan2(l_v*sin(al)-d*cos(th), d*sin(th)+l_v*cos(al));
    if phi < 0, phi = phi + 2*pi; end
    % velocities
    dl_leg = 0.5*(d^2 + l_v^2 - 2*d*l_v*cos(angle))^-0.5 * 2*d*l_v*sin(angle)*(dth - dal);
    y_hip = l_v*sin(al)-d*cos(th);
    x_hip = d*sin(th)+l_v*cos(al);
    dphi = -dth + ((x_hip*(l_v*dal*cos(al)+d*dth*sin(th)) - y_hip*(d*dth*cos(th)-l_v*dal*sin(al))) / x_hip^2) / ((y_hip / x_hip)^2 + 1);
    % accelerations
    ddl_leg = (d*l_v*sin(al - th)*(dal - dth)^2)/(d^2 - 2*sin(al - th)*d*l_v + l_v^2)^(1/2) - (d^2*l_v^2*cos(al - th)^2*(dal - dth)^2)/(d^2 - 2*sin(al - th)*d*l_v + l_v^2)^(3/2) - (d*l_v*cos(al - th)*(ddal - ddth))/(d^2 - 2*sin(al - th)*d*l_v + l_v^2)^(1/2);
    ddphi = ((l_v*cos(al) + d*sin(th))*(l_v*cos(al)*ddal - l_v*sin(al)*dal^2 + d*cos(th)*dth^2 + d*sin(th)*ddth) - (d*cos(th) - l_v*sin(al))*(l_v*cos(al)*dal^2 + l_v*sin(al)*ddal + d*sin(th)*dth^2 - d*cos(th)*ddth))/((l_v*cos(al) + d*sin(th))^2*((d*cos(th) - l_v*sin(al))^2/(l_v*cos(al) + d*sin(th))^2 + 1)) - ddth + (2*((l_v*cos(al) + d*sin(th))*(l_v*cos(al)*dal + d*sin(th)*dth) - (d*cos(th) - l_v*sin(al))*(l_v*sin(al)*dal - d*cos(th)*dth))*(l_v*sin(al)*dal - d*cos(th)*dth))/((l_v*cos(al) + d*sin(th))^3*((d*cos(th) - l_v*sin(al))^2/(l_v*cos(al) + d*sin(th))^2 + 1)) - (((l_v*cos(al) + d*sin(th))*(l_v*cos(al)*dal + d*sin(th)*dth) - (d*cos(th) - l_v*sin(al))*(l_v*sin(al)*dal - d*cos(th)*dth))*((2*(d*cos(th) - l_v*sin(al))^2*(l_v*sin(al)*dal - d*cos(th)*dth))/(l_v*cos(al) + d*sin(th))^3 - (2*(d*cos(th) - l_v*sin(al))*(l_v*cos(al)*dal + d*sin(th)*dth))/(l_v*cos(al) + d*sin(th))^2))/((l_v*cos(al) + d*sin(th))^2*((d*cos(th) - l_v*sin(al))^2/(l_v*cos(al) + d*sin(th))^2 + 1)^2);
 
end
