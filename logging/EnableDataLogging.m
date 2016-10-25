function [  ] = EnableDataLogging( )
    % Definitions
    tg = slrt;
    system_prefix = 'atrias_system/';
    behavior_prefix = 'atrias_control/behavior_control/';
    actuator_prefix = 'atrias_control/actuator_control/';
    % Remove old file scopes
    scopes = tg.getscope();
    if ~isempty(scopes) 
        scopes.stop;
        for i=1:size(scopes,1)
            if strcmp(scopes(i).type, 'File')
                tg.remscope(i);
            end
        end
    end
    % Create scopes
    number_of_fscopes = 2;
    fscopes = [];
    for i=1:number_of_fscopes
        fscopes = [fscopes tg.addscope('file')];
    end
    set(fscopes(1:number_of_fscopes), ...
        {'WriteMode', 'AutoRestart', 'DynamicFileName', 'WriteSize'}, ...
        {'Lazy',      'on',          'on',              2*8192});
    fscopes(1).Filename = 'D:\LOGS\LOG1_<%%%>.dat';
    fscopes(1).Decimation = 4;
    fscopes(1).NumSamples = 1500;
    fscopes(2).Filename = 'D:\LOGS\LOG4_<%%%>.dat';
    fscopes(2).Decimation = 1;
    fscopes(2).NumSamples = 1500;
    % Add signals to scopes
    % 1khz scope
    fscopes(1).addsignal(tg.getsignalid([actuator_prefix 'p1/s1'])); % commanded lateral torques
    fscopes(1).addsignal(tg.getsignalid([actuator_prefix 'p1/s2'])); %
    
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p6/s1'])); % accel x 
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p6/s2'])); % accel y
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p6/s3'])); % accel z
    
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p4/s1'])); % right back leg
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p4/s2'])); % right back gear
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p4/s3'])); % right front leg
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p4/s4'])); % right front gear
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p4/s5'])); % left back leg
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p4/s6'])); % left back gear
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p4/s7'])); % left front leg
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p4/s8'])); % left front gear
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p4/s9'])); % right lateral
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p4/s10'])); % left lateral
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p4/s11'])); % roll 
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p4/s12'])); % yaw 
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p4/s13'])); % pitch 
    
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p5/s1'])); % right back leg
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p5/s2'])); % right back gear
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p5/s3'])); % right front leg
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p5/s4'])); % right front gear
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p5/s5'])); % left back leg
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p5/s6'])); % left back gear
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p5/s7'])); % left front leg
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p5/s8'])); % left front gear
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p5/s9'])); % right lateral
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p5/s10'])); % left lateral
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p5/s11'])); % roll 
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p5/s12'])); % yaw 
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p5/s13'])); % pitch 
    
    fscopes(1).addsignal(tg.getsignalid([behavior_prefix 'p1/s1'])); % desired SEA torques
    fscopes(1).addsignal(tg.getsignalid([behavior_prefix 'p1/s2'])); %
    fscopes(1).addsignal(tg.getsignalid([behavior_prefix 'p1/s3'])); %
    fscopes(1).addsignal(tg.getsignalid([behavior_prefix 'p1/s4'])); %
    
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p14/s1'])); % elmo error codes
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p14/s2'])); %
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p14/s3'])); %
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p14/s4'])); %
    
    fscopes(1).addsignal(tg.getsignalid([system_prefix 'p2/s1'])); % system clock
    
    % 4khz scope
    fscopes(2).addsignal(tg.getsignalid([actuator_prefix 'p2/s1'])); % elmo commanded velocity
    fscopes(2).addsignal(tg.getsignalid([actuator_prefix 'p2/s2'])); %
    fscopes(2).addsignal(tg.getsignalid([actuator_prefix 'p2/s3'])); %
    fscopes(2).addsignal(tg.getsignalid([actuator_prefix 'p2/s4'])); %
    
    fscopes(2).addsignal(tg.getsignalid([system_prefix 'p7/s1'])); % elmo measured position
    fscopes(2).addsignal(tg.getsignalid([system_prefix 'p7/s2'])); % 
    fscopes(2).addsignal(tg.getsignalid([system_prefix 'p7/s3'])); % 
    fscopes(2).addsignal(tg.getsignalid([system_prefix 'p7/s4'])); % 
    
    fscopes(2).addsignal(tg.getsignalid([system_prefix 'p8/s1'])); % elmo measured velocity
    fscopes(2).addsignal(tg.getsignalid([system_prefix 'p8/s2'])); % 
    fscopes(2).addsignal(tg.getsignalid([system_prefix 'p8/s3'])); % 
    fscopes(2).addsignal(tg.getsignalid([system_prefix 'p8/s4'])); % 
    
    fscopes(2).addsignal(tg.getsignalid([system_prefix 'p9/s1'])); % elmo measured torque
    fscopes(2).addsignal(tg.getsignalid([system_prefix 'p9/s2'])); % 
    fscopes(2).addsignal(tg.getsignalid([system_prefix 'p9/s3'])); % 
    fscopes(2).addsignal(tg.getsignalid([system_prefix 'p9/s4'])); % 
    
    fscopes(2).addsignal(tg.getsignalid([system_prefix 'p10/s1'])); % elmo demanded velocity
    fscopes(2).addsignal(tg.getsignalid([system_prefix 'p10/s2'])); % 
    fscopes(2).addsignal(tg.getsignalid([system_prefix 'p10/s3'])); % 
    fscopes(2).addsignal(tg.getsignalid([system_prefix 'p10/s4'])); % 
    
    fscopes(2).addsignal(tg.getsignalid([system_prefix 'p11/s1'])); % elmo demanded torque
    fscopes(2).addsignal(tg.getsignalid([system_prefix 'p11/s2'])); % 
    fscopes(2).addsignal(tg.getsignalid([system_prefix 'p11/s3'])); % 
    fscopes(2).addsignal(tg.getsignalid([system_prefix 'p11/s4'])); % 
    
    fscopes(2).addsignal(tg.getsignalid([system_prefix 'p3/s1'])); % motors enabled
    
    fscopes(2).addsignal(tg.getsignalid([system_prefix 'p2/s1'])); % system clock
    
    % Begin logging data
    fscopes.start();
end
