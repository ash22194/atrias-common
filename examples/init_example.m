%% Example initialization script to load parameters and paths
%%% NOTE: create_core_scopes.m must be run after model is loaded on target PC
close all; clear all; path(pathdef); clc;

% Add all lower directories to path
addpath(genpath(pwd));
addpath(genpath('../atrias-common'));

% Sensor and motor calibration and numerical derivative constants
daq_params;

% Model parameters
model_params;

% Control parameters
actuator_control_params;
%%% application_specific_control_params;
