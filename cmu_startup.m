% CMU_STARTUP Carnegie Mellon University MATLAB startup script.

% Reset environment
close all; clear all; path(pathdef); clc;

currentFolder = fileparts(mfilename('fullpath'));
addpath([currentFolder '/atrias-robot/']);
addpath([currentFolder '/logging/']);
addpath(genpath([currentFolder '/functions/']));
addpath([currentFolder '/scripts/']);

% Set build directory
if ~exist([currentFolder, '/build/'],'dir'), mkdir('build'); end
set_param(0, 'CacheFolder', [currentFolder '/build']);
set_param(0, 'CodeGenFolder', [currentFolder '/build']);

% Set workspace variables
daq_params_cmu;

%% Check system is PC based
% if ispc
%     
%   % Assign ethercat parameters
%   set_param('atrias_system/EtherCAT Init', 'pci_bus', '2');
%   set_param('atrias_system/EtherCAT Init', 'pci_slot', '0');
% 
%   tg = slrt;
%   slrtexplr;
% end 