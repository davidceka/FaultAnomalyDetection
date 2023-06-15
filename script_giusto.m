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




folder = 'Splitted_Fault-NoFault';
fileList = dir(fullfile(folder, '*.mat')); 

%fileList = dir([folder '*.mat']);
disp(fileList)

% Iterate over each file in the directory
for j = 1:length(fileList)
    % Get the file name (including the path)
    filename = fullfile(folder, fileList(j).name);
   
  
   
    
    
    fault_label = int8(0); % if 0 NO FAULT

    % 0 no guasto
    if contains(filename, 'no_fault')
        fault_label = 0;
        dataTable.FaultLabel(j) = fault_label;
    
    %  1 guasto engine
    elseif contains(filename, 'engine_failure')
        fault_label = 1;
        dataTable.FaultLabel(j) = fault_label;
    %  2 aileron destra
    elseif contains(filename, 'left_aileron')
        fault_label = 2;
        dataTable.FaultLabel(j) = fault_label; 

      
    %  3 aileron sinistra
    elseif contains(filename, 'right_aileron')
        fault_label = 3;
        dataTable.FaultLabel(j) = fault_label; 

    

    %  4 aileron entrambi
    elseif contains(filename, 'both_ailerons')
        fault_label = 4;
        dataTable.FaultLabel(j) = fault_label; 
    

    %  5 rudder & aileron posizione 0    
    elseif contains(filename, 'rudder_zero__left_aileron_failure')
        fault_label = 5;
        dataTable.FaultLabel(j) = fault_label;
   
    %  6 rudder sinistra
    elseif contains(filename, 'rudder_left')
        fault_label = 6;
        dataTable.FaultLabel(j) = fault_label;
    % 7 rudder destra
    elseif contains(filename, 'rudder_right')
        fault_label = 7;
        dataTable.FaultLabel(j) = fault_label;

    % 8 elevator posizione 0

    elseif contains(filename, 'elevator_failure')
        fault_label = 8;
        dataTable.FaultLabel(j) = fault_label;

    end
    
    
    
    %% Load the sequence through the constructor
    Sequence = sequence(filename);
    
    %% Print brief information about the sequence
    % Sequence.PrintBriefInfo();
    
    %% For each topic in topics
    topics = fieldnames(Sequence.Topics);
    % Get the start time to normalize times to start from zero
    start_time = Sequence.GetStartTime();
    
   
    for i = 1:numel(topics)
 
    
        % Get the topic name
        topic_name = topics(i);
        
    
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

        if isequal(topic_name{1}, 'mavros_imu_data_raw')
            
            % acc lineare e velocità angolare
            acc_x = [];
            acc_y = [];
            acc_z = [];
            gyro_x = [];
            gyro_y = [];
            gyro_z = [];
            
            % Loop over rows of the table
            for k = 1:size(data, 1)
               
                myStruct_lin = data.linear_acceleration(k);
                myStruct_ang = data.angular_velocity(k);
                
                % Extract values from fields of structure
                acc_x_value = myStruct_lin.x;
                acc_y_value = myStruct_lin.y;
                acc_z_value = myStruct_lin.z;
                gyro_x_value = myStruct_ang.x;
                gyro_y_value = myStruct_ang.y;
                gyro_z_value = myStruct_ang.z;
                
                % Append extracted values to arrays
                acc_x = [acc_x; acc_x_value];
                acc_y = [acc_y; acc_y_value];
                acc_z = [acc_z; acc_z_value];
                gyro_x = [gyro_x; gyro_x_value];
                gyro_y = [gyro_y; gyro_y_value];
                gyro_z = [gyro_z; gyro_z_value];
            end
            topic_imu_data_raw_timetable = timetable(timestamps, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);
        end
    
        if isequal(topic_name{1}, 'mavros_nav_info_velocity')
            % Create timetable for 1 topic with feature selected
            %vel_x_des, vel_y_des, vel_z_des, vel_x, vel_y, vel_z
            topic_velocity_timetable = timetable(timestamps, data.des_x, data.des_y, data.des_z, data.meas_x, data.meas_y, data.meas_z);
            topic_velocity_timetable = renamevars(topic_velocity_timetable, 'Var1', 'vel_x_des');
            topic_velocity_timetable = renamevars(topic_velocity_timetable, 'Var2', 'vel_y_des');
            topic_velocity_timetable = renamevars(topic_velocity_timetable, 'Var3', 'vel_z_des');
            topic_velocity_timetable = renamevars(topic_velocity_timetable, 'Var4', 'vel_x');
            topic_velocity_timetable = renamevars(topic_velocity_timetable, 'Var5', 'vel_y');
            topic_velocity_timetable = renamevars(topic_velocity_timetable, 'Var6', 'vel_z');
        end

        if isequal(topic_name{1}, 'mavros_setpoint_raw_target_global')  
            setpoint_x_err = [];
            setpoint_y_err = [];
            setpoint_z_err = [];
            % Loop over rows of the table
            for k = 1:size(data, 1)
                myStruct_setpoint_pos = data;
                % Extract values from fields of structure
                setpoint_x_value = myStruct_setpoint_pos.latitude(k);
                setpoint_y_value = myStruct_setpoint_pos.longitude(k);
                setpoint_z_value = myStruct_setpoint_pos.altitude(k);

                
                % Append extracted values to arrays
                setpoint_x_err = [setpoint_x_err; setpoint_x_value];
                setpoint_y_err = [setpoint_y_err; setpoint_y_value];
                setpoint_z_err = [setpoint_z_err; setpoint_z_value];
                % Calcolo dell'errore in posizione utilizzando abs
                                
                
            end
            topic_error_setpoint_timetable = timetable(timestamps, setpoint_x_err, setpoint_y_err, setpoint_z_err);
        end
    
        if isequal(topic_name{1}, 'mavros_local_position_pose')   
            gps_x = [];
            gps_y = [];
            gps_z = [];
            % Loop over rows of the table
            for k = 1:size(data, 1)
                myStruct_gps_pos = data.pose(k).position;
                % Extract values from fields of structure
                gps_x_value = myStruct_gps_pos.x;
                gps_y_value = myStruct_gps_pos.y;
                gps_z_value = myStruct_gps_pos.z;

                % Append extracted values to arrays
                gps_x = [gps_x; gps_x_value];
                gps_y = [gps_y; gps_y_value];
                gps_z = [gps_z; gps_z_value];
            end
            topic_local_position_timetable = timetable(timestamps, gps_x, gps_y, gps_z);
    
        end

        if isequal(topic_name{1}, 'mavros_nav_info_roll')
            topic_nav_info_roll_timetable = timetable(timestamps, data.measured, data.commanded);
            topic_nav_info_roll_timetable = renamevars(topic_nav_info_roll_timetable, 'Var1', 'roll');    
            topic_nav_info_roll_timetable = renamevars(topic_nav_info_roll_timetable, 'Var2', 'roll_des');    
        end
    
        if isequal(topic_name{1}, 'mavros_nav_info_pitch')
            topic_nav_info_pitch_timetable = timetable(timestamps, data.measured, data.commanded);
            topic_nav_info_pitch_timetable = renamevars(topic_nav_info_pitch_timetable, 'Var1', 'pitch');    
            topic_nav_info_pitch_timetable = renamevars(topic_nav_info_pitch_timetable, 'Var2', 'pitch_des');    
    
        end
        
        if isequal(topic_name{1}, 'mavros_nav_info_yaw')

            data.err_yaw = abs(data.measured - data.commanded);
    
            % Create timetable 
            topic_err_yaw_timetable = timetable(timestamps, data.err_yaw);
            topic_err_yaw_timetable = renamevars(topic_err_yaw_timetable, 'Var1', 'err_yaw');

            topic_nav_info_yaw_timetable = timetable(timestamps, data.measured, data.commanded);
            topic_nav_info_yaw_timetable = renamevars(topic_nav_info_yaw_timetable, 'Var1', 'yaw');    
            topic_nav_info_yaw_timetable = renamevars(topic_nav_info_yaw_timetable, 'Var2', 'yaw_des');    
        end


        if isequal(topic_name{1}, 'mavros_nav_info_errors')   
            topic_info_errors_timetable = timetable(timestamps, data.xtrack_error, data.aspd_error, data.alt_error);
            topic_info_errors_timetable = renamevars(topic_info_errors_timetable, 'Var1', 'err_track');
            topic_info_errors_timetable = renamevars(topic_info_errors_timetable, 'Var2', 'err_speed');
            topic_info_errors_timetable = renamevars(topic_info_errors_timetable, 'Var3', 'err_alt');
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
            
            %timetablel
                topic_rc_out_timetable = timetable(timestamps, ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8);

            
        end
        
        if isequal(topic_name{1}, 'mavros_nav_info_roll')
    
            data.err_roll = abs(data.measured - data.commanded);
            
            % Create timetable 
            topic_err_roll_timetable = timetable(timestamps, data.err_roll);
            topic_err_roll_timetable = renamevars(topic_err_roll_timetable, 'Var1', 'err_roll');    
        end

        if isequal(topic_name{1}, 'mavros_nav_info_pitch')
       
            data.err_pitch = abs(data.measured - data.commanded);
    
            % Create timetable 
            topic_err_pitch_timetable = timetable(timestamps, data.err_pitch);
            topic_err_pitch_timetable = renamevars(topic_err_pitch_timetable, 'Var1', 'err_pitch');
        end
        
       
    
        if isequal(i, numel(topics)) 
    
            test_timetable = synchronize(topic_error_setpoint_timetable,topic_err_roll_timetable,topic_err_yaw_timetable,topic_err_pitch_timetable,topic_imu_data_raw_timetable, topic_velocity_timetable, topic_local_position_timetable, topic_nav_info_roll_timetable, topic_nav_info_pitch_timetable, topic_nav_info_yaw_timetable, topic_info_errors_timetable, topic_rc_out_timetable, 'regular', 'linear', 'SampleRate', fs_new);

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
    
            acc_x_timetable = timetable(test_timetable.timestamps, test_timetable.acc_x);
            acc_y_timetable = timetable(test_timetable.timestamps, test_timetable.acc_y);
            acc_z_timetable = timetable(test_timetable.timestamps, test_timetable.acc_z);
            gyro_x_timetable = timetable(test_timetable.timestamps, test_timetable.gyro_x);
            gyro_y_timetable = timetable(test_timetable.timestamps, test_timetable.gyro_y);
            gyro_z_timetable = timetable(test_timetable.timestamps, test_timetable.gyro_z);
            
            vel_x_des_timetable = timetable(test_timetable.timestamps, test_timetable.vel_x_des);
            vel_y_des_timetable = timetable(test_timetable.timestamps, test_timetable.vel_y_des);
            vel_z_des_timetable = timetable(test_timetable.timestamps, test_timetable.vel_z_des);
            vel_x_timetable = timetable(test_timetable.timestamps, test_timetable.vel_x);
            vel_y_timetable = timetable(test_timetable.timestamps, test_timetable.vel_y);
            vel_z_timetable = timetable(test_timetable.timestamps, test_timetable.vel_z);
    

            gps_x_timetable = timetable(test_timetable.timestamps, test_timetable.gps_x);
            gps_y_timetable = timetable(test_timetable.timestamps, test_timetable.gps_y);
            gps_z_timetable = timetable(test_timetable.timestamps, test_timetable.gps_z);
    
            roll_timetable = timetable(test_timetable.timestamps, test_timetable.roll);
            roll_des_timetable = timetable(test_timetable.timestamps, test_timetable.roll_des);
    
            pitch_timetable = timetable(test_timetable.timestamps, test_timetable.pitch);
            pitch_des_timetable = timetable(test_timetable.timestamps, test_timetable.pitch_des);

            yaw_timetable = timetable(test_timetable.timestamps, test_timetable.yaw);
            yaw_des_timetable = timetable(test_timetable.timestamps, test_timetable.yaw_des);

            err_track_timetable = timetable(test_timetable.timestamps, test_timetable.err_track);
            err_speed_timetable= timetable(test_timetable.timestamps, test_timetable.err_speed);
            err_al_timetable= timetable(test_timetable.timestamps, test_timetable.err_alt);

            ch1_timetable=timetable(test_timetable.timestamps,test_timetable.ch1);
            ch2_timetable=timetable(test_timetable.timestamps,test_timetable.ch2);
            ch3_timetable=timetable(test_timetable.timestamps,test_timetable.ch3);
            ch4_timetable=timetable(test_timetable.timestamps,test_timetable.ch4);
            ch5_timetable=timetable(test_timetable.timestamps,test_timetable.ch5);
            ch6_timetable=timetable(test_timetable.timestamps,test_timetable.ch6);
            ch7_timetable=timetable(test_timetable.timestamps,test_timetable.ch7);
            ch8_timetable=timetable(test_timetable.timestamps,test_timetable.ch8);

            err_yaw_timetable = timetable(test_timetable.timestamps, test_timetable.err_yaw);
    
            err_pitch_timetable = timetable(test_timetable.timestamps, test_timetable.err_pitch);

            err_roll_timetable = timetable(test_timetable.timestamps, test_timetable.err_roll);

            setpoint_x_timetable = timetable(test_timetable.timestamps, test_timetable.setpoint_x_err);
            setpoint_y_timetable = timetable(test_timetable.timestamps, test_timetable.setpoint_y_err);
            setpoint_z_timetable = timetable(test_timetable.timestamps, test_timetable.setpoint_z_err);

            error_x = abs(gps_x_timetable - setpoint_x_timetable);
            error_y = abs(gps_y_timetable - setpoint_y_timetable);
            error_z = abs(gps_z_timetable - setpoint_z_timetable);

            dataTable.error_x_setpoint(j)={error_x};
            dataTable.error_y_setpoint(j)={error_y};
            dataTable.error_z_setpoint(j)={error_z};

               
            %mavros local position pose
            dataTable.gps_x_timetable(j) = {gps_x_timetable};
            dataTable.gps_y_timetable(j) = {gps_y_timetable};
            dataTable.gps_z_timetable(j) = {gps_z_timetable};
            
            % topic imu data raw
            dataTable.acc_x_timetable(j) = {acc_x_timetable};
            dataTable.acc_y_timetable(j) = {acc_y_timetable};
            dataTable.acc_z_timetable(j) = {acc_z_timetable};
            dataTable.gyro_x_timetable(j) = {gyro_x_timetable};
            dataTable.gyro_y_timetable(j) = {gyro_y_timetable};
            dataTable.gyro_z_timetable(j) = {gyro_z_timetable};
    
            % topic mavros info velocity
            dataTable.velDes_x_timetable(j) = {vel_x_des_timetable};
            dataTable.velDes_y_timetable(j) = {vel_y_des_timetable};
            dataTable.velDes_z_timetable(j) = {vel_z_des_timetable};
            dataTable.vel_x_timetable(j) = {vel_x_timetable};
            dataTable.vel_y_timetable(j) = {vel_y_timetable};
            dataTable.vel_z_timetable(j) = {vel_z_timetable};
            

            % topic mavros info roll
            dataTable.roll_timetable(j) = {roll_timetable}; 
            dataTable.roll_des_timetable(j) = {roll_des_timetable}; 

            % topic mavros info yaw
            dataTable.yaw_timetable(j) = {yaw_timetable}; 
            dataTable.yaw_des_timetable(j) = {yaw_des_timetable}; 
    
            % topic mavros info pitch
            dataTable.pitch_timetable(j) = {pitch_timetable}; 
            dataTable.pitch_des_timetable(j) = {pitch_des_timetable}; 

            %topic mavros nav info errors
            dataTable.err_track_timetable(j) = {err_track_timetable};
            dataTable.err_speed_timetable(j) = {err_speed_timetable};
            dataTable.err_al_timetableT(j) = {err_al_timetable};

            %topic mavros rc out
            dataTable.ch1_timetable(j)={ch1_timetable};
            dataTable.ch2_timetable(j)={ch2_timetable};
            dataTable.ch3_timetable(j)={ch3_timetable};
            dataTable.ch4_timetable(j)={ch4_timetable};
            dataTable.ch5_timetable(j)={ch5_timetable};
            dataTable.ch6_timetable(j)={ch6_timetable};
            dataTable.ch7_timetable(j)={ch7_timetable};
            dataTable.ch8_timetable(j)={ch8_timetable};

            % topic mavros info yaw
            dataTable.err_yaw_timetable(j) = {err_yaw_timetable};
    
            % topic mavros info pitch
            dataTable.err_pitch_timetable(j) = {err_pitch_timetable};

            % topic mavros info roll
            dataTable.err_roll_timetable(j) = {err_roll_timetable};
            
        end
    end

    %if j == 11
    %   return
    % end
end

