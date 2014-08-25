% This contains parameters for the position controller

% Hip and leg saturation parameters
hip_sat = 50;
leg_sat = 50;

% Gains
leg_p = 500;
leg_d = 40;
hip_p = 1500;
hip_d = 10;

% Safety control
sat_estops = false;
