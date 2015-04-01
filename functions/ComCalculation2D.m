function [ x_com, dx_com, y_com, dy_com ] = ComCalculation2D( qRoll, dqRoll, qYaw, dqYaw, qPitch, dqPitch, ...
                                                               l_boom_projected, h_boom, boom_mount_angle, d_y_boom_com_offset )

    % x position
    x_com = l_boom_projected * cos(qRoll) * qYaw - d_y_boom_com_offset * sin(qPitch) * cos(qRoll-boom_mount_angle);

    % x velocity
    dx_com = l_boom_projected*dqYaw*cos(qRoll) - l_boom_projected*dqRoll*sin(qRoll)*qYaw - d_y_boom_com_offset*dqPitch*cos(qPitch)*cos(qRoll - boom_mount_angle) + d_y_boom_com_offset*dqRoll*sin(qPitch)*sin(qRoll - boom_mount_angle);
    
    % y position
    y_com = h_boom + l_boom_projected * sin(qRoll)  -  d_y_boom_com_offset * cos(qRoll-boom_mount_angle) * cos(qPitch);

    % y velocity
    dy_com = l_boom_projected*dqRoll*cos(qRoll) + d_y_boom_com_offset*dqPitch*sin(qPitch)*cos(qRoll - boom_mount_angle) + d_y_boom_com_offset*dqRoll*cos(qPitch)*sin(qRoll - boom_mount_angle);
    
end

