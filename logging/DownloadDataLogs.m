function [ data_struct ] = DownloadDataLogs( tg )
    % download data from target
    prefix = 'F:\LOGS\';
    filename1 = 'LOG1_001.dat';
    filename4 = 'LOG4_001.dat';
    SimulinkRealTime.copyFileToHost(tg, [prefix, filename1]);
    SimulinkRealTime.copyFileToHost(tg, [prefix, filename4]);
    data_1 = SimulinkRealTime.utils.getFileScopeData(filename1);
    data_4 = SimulinkRealTime.utils.getFileScopeData(filename4);
    % name data signals
    data_names_1 = {'lateral_torque_commands', ...
                    'accelerometers', ...
                    'q', ...
                    'dq', ...
                    'desired_sea_torques', ...
                    'elmo_error_codes', ...
                    'time_1000'};
    data_names_4 = {'elmo_commanded_velocities', ...    
                    'elmo_measured_positions', ...
                    'elmo_measured_velocities', ...
                    'elmo_measured_torques', ...
                    'elmo_demanded_velocities', ...
                    'elmo_demanded_torques', ...
                    'motors_enabled',...
                    'time_4000'}; 
    data_sizes_1 = [2,...
                  3,...
                  13,...
                  13,...
                  4,...
                  4,...
                  1];
    data_sizes_4 = [4,...
                  4,...
                  4,...
                  4,...
                  4,...
                  4,...
                  1,...
                  1];
    starts_1 = cumsum(data_sizes_1) - data_sizes_1 + 1;
    ends_1 = cumsum(data_sizes_1);
    starts_4 = cumsum(data_sizes_4) - data_sizes_4 + 1;
    ends_4 = cumsum(data_sizes_4);
    % put data signals in struct
    data_struct = [];
    for i=1:length(data_sizes_1)
        data_struct.([data_names_1{i},'_recorded']) = data_1.data(:,starts_1(i):ends_1(i));
    end
    for i=1:length(data_sizes_4)
        data_struct.([data_names_4{i},'_recorded']) = data_4.data(:,starts_4(i):ends_4(i));
    end
    data_struct.system_clock_recorded = tg.TimeLog;
    data_struct.tet_recorded = tg.TETlog;
    % save file
    controller = tg.Application;
    file_name = sprintf('data/%s - %s',datestr(clock,'mm_dd_yyyy HH_MM_SS_AM'), controller);
    save(file_name, '-struct','data_struct');
end

