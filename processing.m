%% ----------------------------------------------------- 
close all;
clear all;
clc;

%% --------------------------------
% Add the dataset tools library to the path
addpath('alfa-tools');

%% CREAZIONE TABELLA FINALE CON ATTRIBUTI DI INTERESSE
dataTable = table();

fs_new = 25; % New sampling rate


% METTEREMO CICLO ESTERNO
% VARIABILE j scorrerà su tutti i test
% primo test j = 1
% popoliamo tabella finale in riga j perché ogni riga corrisponde ad un
% test

folder = 'Splitted_Fault-NoFault';
fileList = dir(fullfile(folder, '*.mat')); % list all files in folder with .mat extension

%fileList = dir([folder '*.mat']);
disp(fileList)

% Iterate over each file in the directory
for j = 1:length(fileList)
    % Get the file name (including the path)
    filename = fullfile(folder, fileList(j).name);
   
    %j = 1; % rimosso perchè presente ciclo esterno -> j = 1:numel(nomi dei file (test))
  
    %% Input .mat file
    
    % filename = 'processed_MAT_Files/carbonZ_2018-07-18-15-53-31_1_engine_failure.mat';
    
    fault_label = int8(0); % if 0 NO FAULT

    if contains(filename, 'no_fault')
        fault_label = 0;
        dataTable.FaultLabel(j) = fault_label;
    
    elseif contains(filename, 'engine_failure')
        fault_label = 1;
        dataTable.FaultLabel(j) = fault_label;

    elseif contains(filename, 'rudder_zero__left_aileron_failure')
        fault_label = 5;
        dataTable.FaultLabel(j) = fault_label;  

    elseif contains(filename, 'both_ailerons')
        fault_label = 4;
        dataTable.FaultLabel(j) = fault_label; 
    
    %{
    elseif contains(filename, 'left_aileron__right_aileron__failure') 
        % fault_label = 100;    %label provvisoria prima di fare lo split del test
        % dataTable.FaultLabel(j) = fault_label; 
        % carbonZ_2018-09-11-14-52-54_left_aileron__right_aileron__failure
        continue % go to the next for loop

    %}

    elseif contains(filename, 'left_aileron')
        fault_label = 2;
        dataTable.FaultLabel(j) = fault_label; 

    elseif contains(filename, 'right_aileron')
        fault_label = 3;
        dataTable.FaultLabel(j) = fault_label; 

    elseif contains(filename, 'elevator_failure')
        fault_label = 8;
        dataTable.FaultLabel(j) = fault_label;

    elseif contains(filename, 'rudder_right')
        fault_label = 7;
        dataTable.FaultLabel(j) = fault_label;

    elseif contains(filename, 'rudder_left')
        fault_label = 6;
        dataTable.FaultLabel(j) = fault_label;
    end 
    
 
    %{
        0 NO GUASTO
        1 guasto engine
        2 aileron destra
        3 aileron sinistra
        4 aileron entrambi
        5 rudder & aileron posizione 0
        6 rudder sinistra
        7 rudder destra
        8 elevator posizione 0
    %}
    
    
    
    %% Load the sequence through the constructor
    Sequence = sequence(filename);
    
    %% Print brief information about the sequence
    % Sequence.PrintBriefInfo();
    
    %% For each topic in topics
    topics = fieldnames(Sequence.Topics);
    % Get the start time to normalize times to start from zero
    start_time = Sequence.GetStartTime();
    
   
    for i = 1:numel(topics)
 
        % disp(i)
        % disp(numel(topics))
    
        % Get the topic name
        topic_name = topics(i);
        
        %{
        if  isequal(topic_name{1}, 'mavros_imu_data_raw') || isequal(topic_name{1}, 'mavros_imu_atm_pressure') || isequal(topic_name{1}, 'mavros_global_position_compass_hdg') || isequal(topic_name{1}, 'mavctrl_rpy') || isequal(topic_name{1}, 'mavlink_from') ||  isequal(topic_name{1}, 'mavros_local_position_pose') || isequal(topic_name{1}, 'mavros_wind_estimation') || isequal(topic_name{1}, 'mavros_setpoint_raw_target_global') || isequal(topic_name{1}, 'mavros_imu_temperature') || isequal(topic_name{1}, 'diagnostics') || isequal(topic_name{1}, 'mavros_global_position_raw_fix') || isequal(topic_name{1}, 'mavros_state') || isequal(topic_name{1}, 'mavros_rc_in') || isequal(topic_name{1}, 'mavros_setpoint_raw_local') || isequal(topic_name{1}, 'mavros_battery') || isequal(topic_name{1}, 'mavros_global_position_rel_alt') || isequal(topic_name{1}, 'mavros_local_position_odom') || isequal(topic_name{1}, 'mavros_local_position_velocity') || isequal(topic_name{1}, 'mavros_global_position_raw_gps_vel') || isequal(topic_name{1}, 'mavros_rc_out') || isequal(topic_name{1}, 'mavros_time_reference')    
            continue      
        end
        %}


        % Assign data to variable topic 
        topic = Sequence.GetTopicByName(topic_name{1});
    
        % Normalize the time stamps in the topic
        times = topic.Data.time_recv - start_time;
        
        % get data
        data = topic.Data;
    
        % Add column "times" with normalized time
        data.times = times;
        
        if contains(filename, 'no_fault')
           data([data.times] <= 5, :) = [];
           data.times = data.times - 5;
        end
        
        timestamps = seconds(data.times);

        
        if isequal(topic_name{1}, 'mavros_imu_data')
            
            % ------------LINEAR ACCELERATION---------------
            linAcc_x = [];
            linAcc_y = [];
            linAcc_z = [];
            
            % Loop over rows of the table
            for k = 1:size(data, 1)
               
                myStruct = data.linear_acceleration(k);
                
                % Extract values from fields of structure
                linAcc_x_value = myStruct.x;
                linAcc_y_value = myStruct.y;
                linAcc_z_value = myStruct.z;
                
                % Append extracted values to arrays
                linAcc_x = [linAcc_x; linAcc_x_value];
                linAcc_y = [linAcc_y; linAcc_y_value];
                linAcc_z = [linAcc_z; linAcc_z_value];
            end
    
             % ------------ANGULAR VELOCITY---------------
    
            % Extract structure from table
            angVel_x = [];
            angVel_y = [];
            angVel_z = [];
    
            % Loop over rows of the table
            for k = 1:size(data, 1)
                
                % Extract structure from table
                myStruct = data.angular_velocity(k);
                
                % Extract values from fields of structure
                angVel_x_value = myStruct.x;
                angVel_y_value = myStruct.y;
                angVel_z_value = myStruct.z;
                
                % Append extracted values to arrays
                angVel_x = [angVel_x; angVel_x_value];
                angVel_y = [angVel_y; angVel_y_value];
                angVel_z = [angVel_z; angVel_z_value];
            end
    
            topic_imu_data_timetable = timetable(timestamps, linAcc_x, linAcc_y, linAcc_z, angVel_x, angVel_y, angVel_z);
        end
    
    
      
      
    
        if isequal(topic_name{1}, 'mavros_nav_info_velocity')
    
            data.errVel_x = abs(data.des_x - data.meas_x);
            data.errVel_y = abs(data.des_y - data.meas_y);
            data.errVel_z = abs(data.des_z - data.meas_z);
            % data = removevars(data, {'coordinate_frame','des_x','des_y','des_z','header','meas_x','meas_y','meas_z','time_recv'});
           
            % Create timetable for 1 topic with feature selected
            topic_velocity_timetable = timetable(timestamps, data.errVel_x, data.errVel_y, data.errVel_z);
            topic_velocity_timetable = renamevars(topic_velocity_timetable, 'Var1', 'errVel_x');
            topic_velocity_timetable = renamevars(topic_velocity_timetable, 'Var2', 'errVel_y');
            topic_velocity_timetable = renamevars(topic_velocity_timetable, 'Var3', 'errVel_z');
      
        end
    
    
        if isequal(topic_name{1}, 'mavros_global_position_global')   
            topic_global_position_timetable = timetable(timestamps, data.altitude, data.latitude, data.longitude);
            topic_global_position_timetable = renamevars(topic_global_position_timetable, 'Var1', 'altitude');
            topic_global_position_timetable = renamevars(topic_global_position_timetable, 'Var2', 'latitude');
            topic_global_position_timetable = renamevars(topic_global_position_timetable, 'Var3', 'longitude');
    
        end
    
    
        if isequal(topic_name{1}, 'mavros_nav_info_roll')
    
            data.err_roll = abs(data.measured - data.commanded);
            
            % Create timetable 
            topic_err_roll_timetable = timetable(timestamps, data.err_roll);
            topic_err_roll_timetable = renamevars(topic_err_roll_timetable, 'Var1', 'err_roll');    
        end
    
    
    
        if isequal(topic_name{1}, 'mavros_nav_info_airspeed')
    
            data.err_airspeed = abs(data.measured - data.commanded);
            
            % Create timetable 
            topic_err_airspeed_timetable = timetable(timestamps, data.err_airspeed);
            topic_err_airspeed_timetable = renamevars(topic_err_airspeed_timetable, 'Var1', 'err_airspeed');    
    
        end
    
        if isequal(topic_name{1}, 'mavros_nav_info_errors')   
            topic_info_errors_timetable = timetable(timestamps, data.aspd_error, data.alt_error);
            topic_info_errors_timetable = renamevars(topic_info_errors_timetable, 'Var1', 'aspd_error');
            topic_info_errors_timetable = renamevars(topic_info_errors_timetable, 'Var2', 'alt_error');
    
        end
    
    
        if isequal(topic_name{1}, 'mavros_nav_info_yaw')
            data.err_yaw = abs(data.measured - data.commanded);
    
            % Create timetable 
            topic_err_yaw_timetable = timetable(timestamps, data.err_yaw);
            topic_err_yaw_timetable = renamevars(topic_err_yaw_timetable, 'Var1', 'err_yaw');
        end
    
        if isequal(topic_name{1}, 'mavros_nav_info_pitch')
       
            data.err_pitch = abs(data.measured - data.commanded);
    
            % Create timetable 
            topic_err_pitch_timetable = timetable(timestamps, data.err_pitch);
            topic_err_pitch_timetable = renamevars(topic_err_pitch_timetable, 'Var1', 'err_pitch');
        end

        if isequal(topic_name{1}, 'mavros_rc_out')
            ch1 = zeros(size(data.channels,1),1);
            ch2 = zeros(size(data.channels,1),1);
            ch3 = zeros(size(data.channels,1),1);
            ch4 = zeros(size(data.channels,1),1);
            ch5 = zeros(size(data.channels,1),1);
            ch6 = zeros(size(data.channels,1),1);
            ch7 = zeros(size(data.channels,1),1);
            ch8 = zeros(size(data.channels,1),1);

           


            for i = 1:size(data.channels,1)
                ch1(i) = data.channels{i,1};
                ch2(i) = data.channels{i,2};
                ch3(i) = data.channels{i,3};
                ch4(i) = data.channels{i,4};
                ch5(i) = data.channels{i,5};
                ch6(i) = data.channels{i,6};
                ch7(i) = data.channels{i,7};
                ch8(i) = data.channels{i,8};
            end
            
            %timetable
                topic_rc_out_timetable = timetable(timestamps, ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8);

            
        end
        if isequal(topic_name{1}, 'mavros_local_position_pose')
            gps_x = [];
            gps_y = [];
            gps_z = [];
            for k=1:(size(data,1))
                myStruct=data.pose(k);
                gps_x_value=myStruct.position.x;
                gps_y_value=myStruct.position.y;
                gps_z_value=myStruct.position.z;
                
                % Append extracted values to arrays
                gps_x = [gps_x; gps_x_value];
                gps_y = [gps_y; gps_y_value];
                gps_z = [gps_z; gps_z_value];
            end
                topic_local_position_pose_timetable =timetable(timestamps,gps_x,gps_y,gps_z);
                
        end
    
        
      
        
        % if i == 2 % velocity
        % if i == 17 % altitude
        % if i == 17 %roll
        % if i == 29 % pitch
        % if i == 33 % mav_ctrl_path_dev
        % if i == numel(topics) % mavros_time_reference
    
        if isequal(i, numel(topics)) 
    
            test_timetable = synchronize(topic_velocity_timetable,topic_global_position_timetable, topic_imu_data_timetable, topic_err_roll_timetable, topic_err_airspeed_timetable, topic_info_errors_timetable, topic_err_yaw_timetable, topic_err_pitch_timetable,topic_local_position_pose_timetable,topic_rc_out_timetable, 'regular', 'linear', 'SampleRate', fs_new);
                        
            test_timetable = test_timetable([test_timetable.timestamps] >= 0,:);

            if(test_timetable.timestamps(1) ~= 0)
                newRow = test_timetable(1,:);
                test_timetable = [newRow; test_timetable(:,:)];
                test_timetable.timestamps(1) = 0;
            end
            
            num_rows_test_timetable = size(test_timetable, 1);
            
            remain = rem(num_rows_test_timetable, 64);

            if(remain == 0)
                test_timetable = test_timetable(1:end-63,:);
            elseif(remain ~= 1) % vogliamo numero di righe multiplo di 64 + 1 di scarto           
                test_timetable = test_timetable(1:end-(remain-1),:);
            end
            
            dur1 = duration(0, 0, 17.92);  % 0 hours 0 minutes and 17.92 seconds
            if (test_timetable.timestamps(size(test_timetable, 1)) == dur1)
                test_timetable = test_timetable(1:end-(1),:);
            end

            dur2 = duration(0, 0, 35.84);  
            if (test_timetable.timestamps(size(test_timetable, 1)) == dur2)
                test_timetable = test_timetable(1:end-(1),:);
            end
            
            dur3 = duration(0, 0, 71.68);  
            if (test_timetable.timestamps(size(test_timetable, 1)) == dur3)
                test_timetable = test_timetable(1:end-(1),:);
            end

     
            % create timetables to put in the final table
    
            linAcc_xTT = timetable(test_timetable.timestamps, test_timetable.linAcc_x);
            linAcc_yTT = timetable(test_timetable.timestamps, test_timetable.linAcc_y);
            linAcc_zTT = timetable(test_timetable.timestamps, test_timetable.linAcc_z);
            angVel_xTT = timetable(test_timetable.timestamps, test_timetable.angVel_x);
            angVel_yTT = timetable(test_timetable.timestamps, test_timetable.angVel_y);
            angVel_zTT = timetable(test_timetable.timestamps, test_timetable.angVel_z);
    
            errVel_xTT = timetable(test_timetable.timestamps, test_timetable.errVel_x);
            errVel_yTT = timetable(test_timetable.timestamps, test_timetable.errVel_y);
            errVel_zTT = timetable(test_timetable.timestamps, test_timetable.errVel_z);
    
            altitudeTT = timetable(test_timetable.timestamps, test_timetable.altitude);
            latitudeTT = timetable(test_timetable.timestamps, test_timetable.latitude);
            longitudeTT = timetable(test_timetable.timestamps, test_timetable.longitude);
    
            err_roll_timetable = timetable(test_timetable.timestamps, test_timetable.err_roll);
    
            err_airspeed_timetable = timetable(test_timetable.timestamps, test_timetable.err_airspeed);
    
            aspd_error_timetable = timetable(test_timetable.timestamps, test_timetable.aspd_error);
            alt_error_timetable = timetable(test_timetable.timestamps, test_timetable.alt_error);
    
            err_yaw_timetable = timetable(test_timetable.timestamps, test_timetable.err_yaw);
    
            err_pitch_timetable = timetable(test_timetable.timestamps, test_timetable.err_pitch);

            gps_x_timetable=timetable(test_timetable.timestamps,test_timetable.gps_x);
            gps_y_timetable=timetable(test_timetable.timestamps,test_timetable.gps_y);
            gps_z_timetable=timetable(test_timetable.timestamps,test_timetable.gps_z);

            ch1_timetable=timetable(test_timetable.timestamps,test_timetable.ch1);
            ch2_timetable=timetable(test_timetable.timestamps,test_timetable.ch2);
            ch3_timetable=timetable(test_timetable.timestamps,test_timetable.ch3);
            ch4_timetable=timetable(test_timetable.timestamps,test_timetable.ch4);
            ch5_timetable=timetable(test_timetable.timestamps,test_timetable.ch5);
            ch6_timetable=timetable(test_timetable.timestamps,test_timetable.ch6);
            ch7_timetable=timetable(test_timetable.timestamps,test_timetable.ch7);
            ch8_timetable=timetable(test_timetable.timestamps,test_timetable.ch8);

        
            %path_dev_xTT = timetable(test_timetable.timestamps, test_timetable.path_dev_x);
            %path_dev_yTT = timetable(test_timetable.timestamps, test_timetable.path_dev_y);
            %path_dev_zTT = timetable(test_timetable.timestamps, test_timetable.path_dev_z);
    
    
            % put timetables in final table
            
            dataTable.gps_x_timetable(j)={gps_x_timetable};
            dataTable.gps_y_timetable(j)={gps_y_timetable};
            dataTable.gps_z_timetable(j)={gps_z_timetable};

            % topic imu data raw
            dataTable.linAcc_xTT(j) = {linAcc_xTT};
            dataTable.linAcc_yTT(j) = {linAcc_yTT};
            dataTable.linAcc_zTT(j) = {linAcc_zTT};
            dataTable.angVel_xTT(j) = {angVel_xTT};
            dataTable.angVel_yTT(j) = {angVel_yTT};
            dataTable.angVel_zTT(j) = {angVel_zTT};
    
            % topic mavros info velocity
            dataTable.errVel_xTT(j) = {errVel_xTT};
            dataTable.errVel_yTT(j) = {errVel_yTT};
            dataTable.errVel_zTT(j) = {errVel_zTT};
    
            % topic mavros info roll
            dataTable.err_roll_timetable(j) = {err_roll_timetable};
    
            % topic mavros info airspeed
            dataTable.err_airspeed_timetable(j) = {err_airspeed_timetable};
    
            %topic mavros nav info errors
            dataTable.aspd_error_timetable(j) = {aspd_error_timetable};
            dataTable.alt_error_timetable(j) = {alt_error_timetable};
    
            % topic mavros info yaw
            dataTable.err_yaw_timetable(j) = {err_yaw_timetable};
    
            % topic mavros info pitch
            dataTable.err_pitch_timetable(j) = {err_pitch_timetable};

            %topic mavros rc out
            dataTable.ch1_timetable(j)={ch1_timetable};
            dataTable.ch2_timetable(j)={ch2_timetable};
            dataTable.ch3_timetable(j)={ch3_timetable};
            dataTable.ch4_timetable(j)={ch4_timetable};
            dataTable.ch5_timetable(j)={ch5_timetable};
            dataTable.ch6_timetable(j)={ch6_timetable};
            dataTable.ch7_timetable(j)={ch7_timetable};
            dataTable.ch8_timetable(j)={ch8_timetable};
            % topic mvctrl path dev
            %{
            dataTable.path_dev_x(j) = {path_dev_xTT};
            dataTable.path_dev_y(j) = {path_dev_yTT};
            dataTable.path_dev_z(j) = {path_dev_zTT};
            %}
        end
    end

    %if j == 11
    %   return
    % end
end

