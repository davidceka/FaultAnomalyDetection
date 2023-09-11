close all;
clear all;
clc;


addpath('alfa-tools');

folder = 'mat_files';
fileList = dir(fullfile(folder, '*.mat')); % list all files in folder with .mat extension

disp(fileList)

% Iterate over each file in the directory
for j = 1:length(fileList)

    
    filename = fullfile(folder, fileList(j).name);
    Sequence = sequence(filename);
    start_time = Sequence.GetStartTime();
   

    if contains(filename, 'engine_failure')
        time_FailureStart = Sequence.Topics.failure_status_engines.time_recv(1);
        disp(time_FailureStart);

        
        topics = fieldnames(Sequence.Topics);

        for i = 1:numel(topics)
            topic_name = topics{i};     % Get the current field name
            Sequence.Topics.(topic_name)([Sequence.Topics.(topic_name).time_recv] >= time_FailureStart,:) = [];
        end
        [filepath,name,ext] = fileparts(filename);
        name = fullfile(filepath, name); 
        path = strcat('Splitted_Fault-NoFault/', Sequence.Name);
        total_file =  strcat(path, "_no_fault.mat"); 
        save(total_file, 'Sequence');
   
    

     elseif contains(filename, 'elevator_failure')
        time_FailureStart = Sequence.Topics.failure_status_elevator.time_recv(1);
        disp(time_FailureStart);

        
        topics = fieldnames(Sequence.Topics);

        for i = 1:numel(topics)
            topic_name = topics{i};     % Get the current field name
            Sequence.Topics.(topic_name)([Sequence.Topics.(topic_name).time_recv] >= time_FailureStart,:) = [];
        end
        [filepath,name,ext] = fileparts(filename);
        name = fullfile(filepath, name); 
        path = strcat('Splitted_Fault-NoFault/', Sequence.Name);
        total_file =  strcat(path, "_no_fault.mat"); 
        save(total_file, 'Sequence');

        
     
     elseif contains(filename, 'aileron_failure')
        time_FailureStart = Sequence.Topics.failure_status_aileron.time_recv(1);
        disp(time_FailureStart);

        
        topics = fieldnames(Sequence.Topics);

        for i = 1:numel(topics)
            topic_name = topics{i};     % Get the current field name
            Sequence.Topics.(topic_name)([Sequence.Topics.(topic_name).time_recv] >= time_FailureStart,:) = [];
        end
        [filepath,name,ext] = fileparts(filename);
        name = fullfile(filepath, name); 
        path = strcat('Splitted_Fault-NoFault/', Sequence.Name);
        total_file =  strcat(path, "_no_fault.mat"); 
        save(total_file, 'Sequence');

        
      

      elseif contains(filename, 'aileron__failure')
        time_FailureStart = Sequence.Topics.failure_status_aileron.time_recv(1);
        disp(time_FailureStart);

        
        topics = fieldnames(Sequence.Topics);

        for i = 1:numel(topics)
            topic_name = topics{i};     % Get the current field name
            Sequence.Topics.(topic_name)([Sequence.Topics.(topic_name).time_recv] >= time_FailureStart,:) = [];
        end
        [filepath,name,ext] = fileparts(filename);
        name = fullfile(filepath, name); 
        path = strcat('Splitted_Fault-NoFault/', Sequence.Name);
        total_file =  strcat(path, "_no_fault.mat"); 
        save(total_file, 'Sequence');

        

     elseif contains(filename, 'ailerons_failure')
        time_FailureStart = Sequence.Topics.failure_status_aileron.time_recv(1);
        disp(time_FailureStart);

        
        topics = fieldnames(Sequence.Topics);

        for i = 1:numel(topics)
            topic_name = topics{i};     % Get the current field name
            Sequence.Topics.(topic_name)([Sequence.Topics.(topic_name).time_recv] >= time_FailureStart,:) = [];
        end
        [filepath,name,ext] = fileparts(filename);
        name = fullfile(filepath, name); 
        path = strcat('Splitted_Fault-NoFault/', Sequence.Name);
        total_file =  strcat(path, "_no_fault.mat"); 
        save(total_file, 'Sequence');

        


     elseif contains(filename, 'rudder_right')
        time_FailureStart = Sequence.Topics.failure_status_rudder.time_recv(1);
        disp(time_FailureStart);

        
        topics = fieldnames(Sequence.Topics);

        for i = 1:numel(topics)
            topic_name = topics{i};     % Get the current field name
            Sequence.Topics.(topic_name)([Sequence.Topics.(topic_name).time_recv] >= time_FailureStart,:) = [];
        end
        [filepath,name,ext] = fileparts(filename);
        name = fullfile(filepath, name); 
        path = strcat('Splitted_Fault-NoFault/', Sequence.Name);
        total_file =  strcat(path, "_no_fault.mat"); 
        save(total_file, 'Sequence');

        

     elseif contains(filename, 'rudder_left')
        time_FailureStart = Sequence.Topics.failure_status_rudder.time_recv(1);
        disp(time_FailureStart);

        
        topics = fieldnames(Sequence.Topics);

        for i = 1:numel(topics)
            topic_name = topics{i};     % Get the current field name
            Sequence.Topics.(topic_name)([Sequence.Topics.(topic_name).time_recv] >= time_FailureStart,:) = [];
        end
        [filepath,name,ext] = fileparts(filename);
        name = fullfile(filepath, name); 
        path = strcat('Splitted_Fault-NoFault/', Sequence.Name);
        total_file =  strcat(path, "_no_fault.mat"); 
        save(total_file, 'Sequence');

       

     else
        continue
     end 
    
 
end
