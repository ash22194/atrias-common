%% Save time, q, dq, and torque commands for the current target model
time_recorded = tg.TimeLog;
data = tg.OutputLog;
data(isnan(tg.OutputLog)) = 0;
q_recorded = data(:,1:13);
dq_recorded = data(:,14:26);
accelerometer_recorded = data(:,27:29);
tau_recorded = data(:,30:35);
velocity_mode_recorded = data(:,36:39);
velocity_command_recorded = data(:,40:43);
elmo_velocity_recorded = data(:,44:47);
elmo_torque_recorded = data(:,48:51); 
desired_sea_torques_recorded = data(:,52:55);
elmo_current_commands_recorded = data(:,56:59);
lateral_current_commands_recorded = data(:,60:61);
elmo_torque_demands_recorded = data(:,62:65);
motors_enabled_recorded = data(:,66);
qact = data(:,67:70);
qd = data(:,71:74);
tet_recorded = tg.TETlog;
controller = tg.Application;
file_name = sprintf('data/%s - %s',datestr(clock,'mm_dd_yyyy HH_MM_SS_AM'), controller);
save(file_name, 'time_recorded','q_recorded','dq_recorded','accelerometer_recorded','tau_recorded','velocity_command_recorded','velocity_mode_recorded','elmo_velocity_recorded','elmo_torque_recorded','desired_sea_torques_recorded','elmo_current_commands_recorded','lateral_current_commands_recorded','elmo_torque_demands_recorded','motors_enabled_recorded','tet_recorded','qact','qd');