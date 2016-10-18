%% Load data into workspace
t = time_recorded;
t_start = 10;
t_end =  time_recorded(end);
t_select = t>=t_start & t<=t_end;
q = q_recorded(t_select,:);
dq = dq_recorded(t_select,:);
tau_lateral = lateral_torque_commands_recorded(t_select,:);
t = time_recorded(t_select);
qinc = elmo_measured_positions_recorded(t_select,:);
dqinc = elmo_measured_velocities_recorded(t_select,:);
velocity_command = sagittal_velocity_commands_recorded(t_select,:);
elmo_velocity = elmo_measured_velocities_recorded(t_select,:);
elmo_torque = elmo_measured_torques_recorded(t_select,:);
desired_sea_torques = desired_sea_torques_recorded(t_select,:);
elmo_torque_demands = elmo_demanded_torques_recorded(t_select,:);
elmo_velocity_demands = elmo_demanded_velocities_recorded(t_select,:);
motors_enabled = motors_enabled_recorded(t_select,:);
motors_on = [0; diff(motors_enabled)] == 1;
motors_off = [diff(motors_enabled); 0] == -1;
fprintf('Motors enabled at t=%f s\n',t(motors_on));
fprintf('Motors disabled at t=%f s\n',t(motors_off));

%% Start plotting
% Leg Segment Positions
plot_fscope('Leg Position',{t,t}, {q(:,1:4)*180/pi, q(:,5:8)*180/pi},...
    {{'qBackLoad';'qBackGear';'qFrontLoad';'qFrontGear'},{'qBackLoad';'qBackGear';'qFrontLoad';'qFrontGear'}},...
    {'Right Leg Position','Left Leg Position'},{'Time (sec)','Time (sec)'},{'Angle (degrees)','Angle (degrees)'},{'auto','auto'});
% Leg Segment Velocities
plot_fscope('Leg Velocity',{t,t}, {dq(:,1:4)*180/pi, dq(:,5:8)*180/pi},...
    {{'dqBackLoad';'dqBackGear';'dqFrontLoad';'dqFrontGear'},{'dqBackLoad';'dqBackGear';'dqFrontLoad';'dqFrontGear'}},...
    {'Right Leg Velocity','Left Leg Velocity'},{'Time (sec)','Time (sec)'},...
    {'Velocity (degrees / sec)','Velocity (degrees / sec)'},{'auto','auto'},2,[]);
% Fourbar Leg Geometry
right_leg_length = cos((q(:,1)-q(:,3))/2);
left_leg_length = cos((q(:,5)-q(:,7))/2);
right_leg_length_using_motors = cos((q(:,2)-q(:,4))/2);
left_leg_length_using_motors = cos((q(:,6)-q(:,8))/2);
right_leg_width = sin((q(:,1)-q(:,3))/2);
left_leg_width = sin((q(:,5)-q(:,7))/2);
plot_fscope('Virtual Leg Length',{t,t}, {[right_leg_length right_leg_length_using_motors], [left_leg_length left_leg_length_using_motors]},...
    {{'Measured','Using Motor Position'},{'Measured','Using Motor Position'}},...
    {'Right Leg Length','Left Leg Length'},{'Time (sec)','Time (sec)'},...
    {'Length (m)','Length (m)'},{[0, 1],[0, 1]},2,[]);
% Virtual Leg Angle
right_leg_angle = (q(:,1)+q(:,3))/2;
left_leg_angle = (q(:,5)+q(:,7))/2;
right_leg_angle_velocity = (dq(:,1)+dq(:,3))/2;
left_leg_angle_velocity = (dq(:,5)+dq(:,7))/2;
plot_fscope('Virtual Leg Angle',{t,t}, {right_leg_angle*180/pi, left_leg_angle*180/pi},...
    {{'Measured'},{'Measured'}},...
    {'Right Leg Angle','Left Leg Angle'},{'Time (sec)','Time (sec)'},...
    {'Angle (degrees)','Angle (degrees)'},{[90, 270],[90, 270]},2,[]);
% Lateral Positions)
plot_fscope('Lateral Position',{t,t}, {[(q(:,9:10))*180/pi], [dq(:,9:10)*180/pi]},...
{{'Right Inc Position';'Left Inc Position'},{'Right Inc Velocity','Left Inc Velocity'}},...
{'Lateral Angles','Lateral Velocities'},{'Time (sec)','Time (sec)'},{'Angle (degrees)','Velocity (degrees/s)'},{'auto','auto'},2);

% IMU data
torso_pitch = q(:,13);
torso_roll = q(:,11);
right_leg_roll = pi/2 - q(:,5);
left_leg_roll = pi/2 - q(:,6);

% SEA deflections
deflections = [ q(:,2)-q(:,1), q(:,4)-q(:,3), q(:,6)-q(:,5), q(:,8)-q(:,7)];
spring_forces = zeros(size(deflections));
for i=1:size(deflections,1)
    spring_forces(i,:) = SeaSpringForce(deflections(i,:)')';
end
plot_fscope('SEA Torques',{t,t}, {[spring_forces(:,1:2), desired_sea_torques(:,1:2)], [spring_forces(:,3:4), desired_sea_torques(:,3:4)]}, ...
    {{'Back Measured','Front Measured','Back Cmd','Front Cmd'},{'Back Measured','Front Measured','Back Cmd','Front Cmd'}},...
    {'Right Torques','Left Torques'}, {'Time (sec)','Time (sec)'}, {'Torque (Nm)','Torque (Nm)'}, {'auto','auto'}, 2);
% commanded lateral motor torques
lateral_torque_commands = [tau_lateral(:,1) tau_lateral(:,2)];
plot_fscope('Lateral Torques',{t}, {lateral_torque_commands}, ...
    {{'Right Commanded','Left Commanded'}},...
    {'Lateral Torques'}, {'Time (sec)'}, {'Torque (Nm)'}, {[-700 250]}, 1, []);
% leg force
    right_leg_force_measured = (spring_forces(:,2)-spring_forces(:,1)) ./ right_leg_width;
    left_leg_force_measured = (spring_forces(:,4)-spring_forces(:,3)) ./ left_leg_width;
plot_fscope('Leg Forces',{t,t}, {[right_leg_force_measured], [left_leg_force_measured]}, ...
    {{'Measured'},{'Measured'}},...
    {'Right Leg Force','Left Leg Force'}, {'Time (sec)','Time (sec)'}, {'Force (N)','Force (N)'}, {[-20 1800],[-20 1800]}, 2);
% leg torque
    right_leg_torque_measured = (spring_forces(:,1)+spring_forces(:,2));
    left_leg_torque_measured = (spring_forces(:,3)+spring_forces(:,4));
plot_fscope('Leg Torques',{t,t}, {[right_leg_torque_measured], [left_leg_torque_measured]}, ...
    {{'Measured'},{'Measured'}},...
    {'Right Leg Torque','Left Leg Torque'}, {'Time (sec)','Time (sec)'}, {'Torque (Nm)','Torque (Nm)'}, {[-700,700],[-700,700]}, 2);

% Velocity commands
plot_fscope('Motor Velocity',{t,t}, {[velocity_command(:,1:2), elmo_velocity_demands(:,1:2), elmo_velocity(:,1:2)], [velocity_command(:,3:4), elmo_velocity_demands(:,3:4), elmo_velocity(:,3:4)]}, ...
    {{'Back Cmd','Front Cmd','Back Dmd','Front Dmd','Back Measured','Front Measured'},{'Back Cmd','Front Cmd','Back Dmd','Front Dmd','Back Measured','Front Measured'}},...
    {'Right Motor Velocities','Left Motor Velocities'}, {'Time (sec)','Time (sec)'}, {'rad/s','rad/s'}, {'auto','auto'}, 2);
plot_fscope('Motor Velocity',{t,t}, {[dq(:,[2 4]), elmo_velocity(:,1:2)], [dq(:,[6 8]), elmo_velocity(:,3:4)]}, ...
    {{'Back Abs','Front Abs','Back Inc','Front Inc'},{'Back Abs','Front Abs','Back Inc','Front Inc'}},...
    {'Right Motor Velocities','Left Motor Velocities'}, {'Time (sec)','Time (sec)'}, {'rad/s','rad/s'}, {'auto','auto'}, 2);

% Torque commands
plot_fscope('Motor Torque',{t,t}, {[elmo_torque(:,1:2), elmo_torque_demands(:,1:2)], [elmo_torque(:,3:4), elmo_torque_demands(:,3:4)]}, ...
    {{'Back Measured','Front Measured', 'Back Dmd','Front Dmd'},{'Back Measured','Front Measured', 'Back Dmd','Front Dmd'}},...
    {'Right Motor Torques','Left Motor Torques'}, {'Time (sec)','Time (sec)'}, {'N','N'}, {'auto','auto'}, 2);           
