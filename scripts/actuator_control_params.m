% SEA control params
max_sagittal_loaded_torque = min(MTR_MAX_CURRENT, LEG_CURRENT_LIMIT)*LEG_MOTOR_CONSTANT*LEG_MTR_GEAR_RATIO;
max_sagittal_unloaded_torque = 3*178.5; % Nm
%-- Low Pass filter for acceleration
fcut_acceleration = 60*(2*pi); % Hz, Low pass filter cutoff frequency when calculating acceleration from velocity
lpf_damping = sqrt(2)/2; % butterworth damping ratio
B1_lpf_accel = -2*exp(-lpf_damping*fcut_acceleration*sample_time)*cos(fcut_acceleration*sample_time*sqrt(1-lpf_damping^2));
B2_lpf_accel = exp(-2*lpf_damping*fcut_acceleration*sample_time);
A_lpf_accel = 1 + B1_lpf_accel + B2_lpf_accel;
%-- Low Pass filter for torque derivative
fcut_dtau = 100*(2*pi); % Hz, Low pass filter cutoff frequency when calculating acceleration from velocity
lpf_damping = sqrt(2)/2; % butterworth damping ratio
B1_lpf_dtau = -2*exp(-lpf_damping*fcut_dtau*elmo_sample_time)*cos(fcut_dtau*elmo_sample_time*sqrt(1-lpf_damping^2));
B2_lpf_dtau = exp(-2*lpf_damping*fcut_dtau*elmo_sample_time);
A_lpf_dtau = 1 + B1_lpf_dtau + B2_lpf_dtau;
%-- Low Pass filter to reduce resonance
fcut_res = 20*(2*pi); % Hz, Low pass filter cutoff frequency when calculating acceleration from velocity
lpf_damping = 0.5;
B1_lpf_res = -2*exp(-lpf_damping*fcut_res*sample_time)*cos(fcut_res*sample_time*sqrt(1-lpf_damping^2));
B2_lpf_res = exp(-2*lpf_damping*fcut_res*sample_time);
A_lpf_res = 1 + B1_lpf_res + B2_lpf_res;
%-- Notch filter for output (flight 1)
notch_damping = 0.1;
fnotch_f1 = 27*(2*pi);
B1_notch_f1 = -2*exp(-notch_damping*fnotch_f1*sample_time)*cos(fnotch_f1*sample_time*sqrt(1-notch_damping^2));
B2_notch_f1 = exp(-2*notch_damping*fnotch_f1*sample_time);
A1_notch_f1 = -2*cos(fnotch_f1*sample_time);
A2_notch_f1 = 1;
K_notch_f1 = (1+B1_notch_f1+B2_notch_f1)/(1+A1_notch_f1+A2_notch_f1);
%-- PD gains
kp_sea_torque = 0.60*[0.3, 0.3, 6.0, 6.0];
gain_nodes_sea_torque = [0,65,350,700];
kd_sea_percent_of_critical_damping = 0.70;

%-- Feedforward
sea_sliding_friction = [0 0 0 0]; % Nm/(rad/s)
sea_offset_friction = [0 0 0 0]; % Nm
sea_coulomb_friction = [30 20 20 20]; % Nm
sea_coulomb_friction_direction_switch_sensitivity = 20*[1 1 1 1];

%-- Thresholds for activating SEA control
loaded_unloaded_torque_threshold = 65*ones(1,4); % Nm, minimum measured torque to use large PD gains
sea_control_torque_cmd_small_threshold = 1*ones(1,4); %Nm, minimum commanded torque to activate sea control
%-- Friction Compensation Model for sagittal motors
karnopp_model_velocity_deadzone = [0.15 0.15 0.15 0.15]'; % rad/s
karnopp_model_coulomb_friction = [35 20 20 20]';
karnopp_model_static_friction = [50 40 35 35]';
karnopp_model_integration_constant = 1/sample_time;
karnopp_model_recursive_filter_sample_number = 0.050 / sample_time;

%-- Low pass filter for IMU measurements
fcut_imu = 250*(2*pi);
lpf_damping = sqrt(2)/2;
B1_lpf_imu = -2*exp(-lpf_damping*fcut_imu*sample_time)*cos(fcut_imu*sample_time*sqrt(1-lpf_damping^2));
B2_lpf_imu = exp(-2*lpf_damping*fcut_imu*sample_time);
A_lpf_imu = 1 + B1_lpf_imu + B2_lpf_imu;
fcut_accelerometer = 80*(2*pi);
lpf_damping = sqrt(2)/2;
B1_lpf_accelerometer = -2*exp(-lpf_damping*fcut_accelerometer*sample_time)*cos(fcut_accelerometer*sample_time*sqrt(1-lpf_damping^2));
B2_lpf_accelerometer = exp(-2*lpf_damping*fcut_accelerometer*sample_time);
A_lpf_accelerometer = 1 + B1_lpf_accelerometer + B2_lpf_accelerometer;

%% SEA Kalman Filter
% Model params
Ir = j_rotor;
It = i_torso(2);
Il_front = mean(j_segments([2, 4]));
Il_back = mean(j_segments([1, 3]));
k_front = mean(k_sea_low([2, 4]));
k_back = mean(k_sea_low([1, 3]));
N = LEG_MTR_GEAR_RATIO;
sea_kalman_sample_time = elmo_sample_time;
% Filter model
A_kalman_sea_swing_front = [1, sea_kalman_sample_time, 0, 0; ...
                            0*k_front*sea_kalman_sample_time*-(Ir+It-Ir*N)/(Ir*It*N^2), 1, 0*k_front*sea_kalman_sample_time*(Ir+It-Ir*N)/(Ir*It*N^2), 0; ...
                            0, 0, 1, sea_kalman_sample_time; ...
                            0*k_front*sea_kalman_sample_time*(1/Il_front + (N-1)/(N*It)), 0, 0*k_front*sea_kalman_sample_time*-(1/Il_front + (N-1)/(N*It)), 1];
A_kalman_sea_swing_back = [1, sea_kalman_sample_time, 0, 0; ...
                            0*k_back*sea_kalman_sample_time*-(Ir+It-Ir*N)/(Ir*It*N^2), 1, 0*k_back*sea_kalman_sample_time*(Ir+It-Ir*N)/(Ir*It*N^2), 0; ...
                            0, 0, 1, sea_kalman_sample_time; ...
                            0*k_back*sea_kalman_sample_time*(1/Il_back + (N-1)/(N*It)), 0, 0*k_back*sea_kalman_sample_time*-(1/Il_back + (N-1)/(N*It)), 1];
B_kalman_sea_swing_front = [[0; (Ir+It)/(Ir*It*N); 0; 1/It], ...
                            [0; -(Ir+It-Ir*N)/(Ir*It*N^2); 0; (1/Il_front + (N-1)/(N*It))]]*sea_kalman_sample_time;
B_kalman_sea_swing_back = [[0; (Ir+It)/(Ir*It*N); 0; 1/It], ...
                            [0; -(Ir+It-Ir*N)/(Ir*It*N^2); 0; (1/Il_back + (N-1)/(N*It))]]*sea_kalman_sample_time;
C_kalman_sea_swing = [1, 0, 0, 0; ...
                      1, 0, 0, 0; ...
                      0, 0, 1, 0];
G_kalman_sea_swing_front = [[0; (Ir+It)/(Ir*It*N); 0; 1/It], ...
                            [0; (-1)/(It*N); 0; 0], ...
                            [0; -(Ir+It-Ir*N)/(Ir*It*N^2); 0; (1/Il_front + (N-1)/(N*It))]];
G_kalman_sea_swing_back = [[0; (Ir+It)/(Ir*It*N); 0; 1/It], ...
                           [0; (-1)/(It*N); 0; 0], ...
                           [0; -(Ir+It-Ir*N)/(Ir*It*N^2); 0; (1/Il_back + (N-1)/(N*It))]];
% Covariances
Q_kalman_rotor_torque = 20^2;%2^2;
Q_kalman_trunk_torque = 30^2;
Q_kalman_load_torque = 20^2;%5^2;
R_kalman_absolute_fail = (90*pi/180)^2;
R_kalman_absolute_normal = (1e-1*pi/180)^2;
R_kalman_incremental_unloaded = (0*pi/180)^2 + (LEG_INC_ENCODER_RAD_PER_TICK)^2;
R_kalman_incremental_loaded = (1.7*pi/180)^2 + (LEG_INC_ENCODER_RAD_PER_TICK)^2;
P0_kalman_sea_swing = diag([R_kalman_absolute_normal, 2*R_kalman_absolute_normal/sample_time^2,R_kalman_absolute_normal, 2*R_kalman_absolute_normal/sample_time^2]);
kalman_load_to_deflect_transmission = 600;
% Low pass for creating velocities from positions
fcut_sea_kalman_velocities = 250*(2*pi);
lpf_damping = sqrt(2)/2;
B1_lpf_sea_kalman_velocities = -2*exp(-lpf_damping*fcut_sea_kalman_velocities*sea_kalman_sample_time)*cos(fcut_sea_kalman_velocities*sea_kalman_sample_time*sqrt(1-lpf_damping^2));
B2_lpf_sea_kalman_velocities = exp(-2*lpf_damping*fcut_sea_kalman_velocities*sea_kalman_sample_time);
A_lpf_sea_kalman_velocities = 1 + B1_lpf_sea_kalman_velocities + B2_lpf_sea_kalman_velocities;
% Swing trajectory testing
test_amplitude = 5*pi/180;
test_freq = 0.5*2*pi;

