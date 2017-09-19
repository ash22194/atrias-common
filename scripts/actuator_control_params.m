%% SEA control params

%-- Low Pass filter for converting 1khz to 4khz commands
fcut_1to4 = 500*(2*pi); 
lpf_damping = sqrt(2)/2; 
B1_lpf_1to4 = -2*exp(-lpf_damping*fcut_1to4*elmo_sample_time)*cos(fcut_1to4*elmo_sample_time*sqrt(1-lpf_damping^2));
B2_lpf_1to4 = exp(-2*lpf_damping*fcut_1to4*elmo_sample_time);
A_lpf_1to4 = 1 + B1_lpf_1to4 + B2_lpf_1to4;

%-- Low Pass filter for torque derivative
fcut_dtau = 100*(2*pi); % Hz, Low pass filter cutoff frequency when calculating acceleration from velocity
lpf_damping = sqrt(2)/2; % butterworth damping ratio
B1_lpf_dtau = -2*exp(-lpf_damping*fcut_dtau*elmo_sample_time)*cos(fcut_dtau*elmo_sample_time*sqrt(1-lpf_damping^2));
B2_lpf_dtau = exp(-2*lpf_damping*fcut_dtau*elmo_sample_time);
A_lpf_dtau = 1 + B1_lpf_dtau + B2_lpf_dtau;

%-- SEA Kalman Filter
% Model params
model_params;
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


% SEA Feedforward
Q_kalman_sea_ff = (0.002/sea_kalman_sample_time)^2; % input: motor_acceleration (rad/s^2)
R_kalman_sea_ff = 0.2^2; % measurement: motor_velocity (rad/s)
