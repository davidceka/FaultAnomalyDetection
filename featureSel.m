
function [featureTable,outputTable] = diagnosticFeatures(inputData)
%DIAGNOSTICFEATURES recreates results in Diagnostic Feature Designer.
%
% Input:
%  inputData: A table or a cell array of tables/matrices containing the
%  data as those imported into the app.
%
% Output:
%  featureTable: A table containing all features and condition variables.
%  outputTable: A table containing the computation results.

% Create output ensemble.
features = ["gps_x_timetable";"gps_y_timetable";"gps_z_timetable";"acc_x_timetable";"acc_y_timetable";"acc_z_timetable";"gyro_x_timetable";"gyro_y_timetable";"gyro_z_timetable";"velDes_x_timetable";"velDes_y_timetable";"velDes_z_timetable";"vel_x_timetable";"vel_y_timetable";"vel_z_timetable";"roll_timetable";"roll_des_timetable";"yaw_timetable";"yaw_des_timetable";"pitch_timetable";"pitch_des_timetable";"err_track_timetable";"err_speed_timetable";"err_al_timetableT";"ch1_timetable";"ch2_timetable";"ch3_timetable";"ch4_timetable";"ch5_timetable";"ch6_timetable";"ch7_timetable";"ch8_timetable"];
%outputEnsemble = workspaceEnsemble(inputData,'DataVariables', features,'ConditionVariables',"FaultLabel");
outputEnsemble = workspaceEnsemble(inputData,'DataVariables', features,'ConditionVariables',"FaultLabel");

% Reset the ensemble to read from the beginning of the ensemble.
reset(outputEnsemble);

% Append new frame policy name to DataVariables.
outputEnsemble.DataVariables = [outputEnsemble.DataVariables;"FRM_1"];

% Set SelectedVariables to select variables to read from the ensemble.
outputEnsemble.SelectedVariables = features;

% Loop through all ensemble members to read and write data.
while hasdata(outputEnsemble)
    % Read one member.
    member = read(outputEnsemble);
    list_lower_bounds = [];
    list_upper_bounds = [];
    for f = 1:length(features)
        feature = features(f); % costruisce il nome completo della variabile
        feature_full = sprintf('%s_full', feature);

        eval(sprintf('%s = readMemberData(member, feature ,["Time","Var1"]);', feature_full));
        list_lower_bounds = [list_lower_bounds, eval([feature_full '.Time(1)'])];
        list_upper_bounds = [list_upper_bounds, eval([feature_full '.Time(end)'])];
    end
    lowerBound = min(list_lower_bounds);
    upperBound = max(list_upper_bounds);
    fullIntervals = frameintervals([lowerBound upperBound],2.56,2.56,'FrameUnit',"seconds");
    intervals = fullIntervals;

    % Initialize a table to store frame results.
    frames = table;

    % Loop through all frame intervals and compute results.
    for ct = 1:height(intervals)
        % Get all input variables.
        
        % Initialize a table to store results for one frame interval.
        frame = intervals(ct,:);
        
        for i = 1:height(features)
            feature= features(i); 
            feature = sprintf('%s', feature);
            feature_full = sprintf('%s_full', feature); 
            features_full_time = eval([feature_full '.Time' ]);
            feature_table = eval(feature_full);
            eval(sprintf("%s = feature_table((features_full_time  >=intervals{ct,1})&(features_full_time<intervals{ct,2}),:);", feature));  
        
            %% SignalFeatures
            try
                % Compute signal features.
                inputSignal =  eval([feature '.Var1']);
                ClearanceFactor = max(abs(inputSignal))/(mean(sqrt(abs(inputSignal)))^2);
                CrestFactor = peak2rms(inputSignal);
                ImpulseFactor = max(abs(inputSignal))/mean(abs(inputSignal));
                Kurtosis = kurtosis(inputSignal);
                Mean = mean(inputSignal,'omitnan');
                PeakValue = max(abs(inputSignal));
                RMS = rms(inputSignal,'omitnan');
                ShapeFactor = rms(inputSignal,'omitnan')/mean(abs(inputSignal),'omitnan');
                Skewness = skewness(inputSignal);
                Std = std(inputSignal,'omitnan');
    
                % Concatenate signal features.
                featureValues = [ClearanceFactor,CrestFactor,ImpulseFactor,Kurtosis,Mean,PeakValue,RMS,ShapeFactor,Skewness,Std];
                % Package computed features into a table.
                featureNames = {'ClearanceFactor','CrestFactor','ImpulseFactor','Kurtosis','Mean','PeakValue','RMS','ShapeFactor','Skewness','Std'};
                % Package computed features into a table.
                feature_sigstats = sprintf('%s_sigstats', feature);
                eval(sprintf("%s = array2table(featureValues,'VariableNames',featureNames);", feature_sigstats));  
            
            catch
                % Package computed features into a table.
                featureValues = NaN(1,10);
                featureNames = {'ClearanceFactor','CrestFactor','ImpulseFactor','Kurtosis','Mean','PeakValue','RMS','ShapeFactor','Skewness','Std'};
                feature_sigstats = sprintf('%s_sigstats', feature);
                eval(sprintf("%s = array2table(featureValues,'VariableNames',featureNames);", feature_sigstats));  
            end
    
            % Append computed results to the frame table.
            frame = [frame, ...
                    table({eval(feature_sigstats)},'VariableNames',{feature_sigstats})];  
         
            %% PowerSpectrum
            try
                % Get units to use in computed spectrum.
                tuReal = "seconds";
    
                % Compute effective sampling rate.
                array_time = eval([feature '.Time' ]);
                tNumeric = time2num(array_time,tuReal);
                [Fs,irregular] = effectivefs(tNumeric);
                Ts = 1/Fs;
                % Resample non-uniform signals.
                x_raw = eval([feature '.Var1' ]);
                if irregular
                    x = resample(x_raw,tNumeric,Fs,'linear');
                else
                    x = x_raw;
                end
    
                % Set Welch spectrum parameters.
                L = fix(length(x)/4.5);
                noverlap = fix(L*50/100);
                win = hamming(L);
    
                % Compute the power spectrum.
                [ps,f] = pwelch(x,win,noverlap,[],Fs);
                w = 2*pi*f;
    
                % Convert frequency unit.
                factor = funitconv('rad/TimeUnit', 'Hz', 'seconds');
                w = factor*w;
                Fs = 2*pi*factor*Fs;
    
                % Remove frequencies above Nyquist frequency.
                I = w<=(Fs/2+1e4*eps);
                w = w(I);
                ps = ps(I);
    
                % Configure the computed spectrum.
                ps = table(w, ps, 'VariableNames', {'Frequency', 'SpectrumData'});
                ps.Properties.VariableUnits = {'Hz', ''};
                ps = addprop(ps, {'SampleFrequency'}, {'table'});
                ps.Properties.CustomProperties.SampleFrequency = Fs;
                feature_ps=sprintf('%s_ps', feature);
                eval(sprintf("%s = ps;", feature_ps));  
            catch
                feature_ps=sprintf('%s_ps', feature);
                eval(sprintf("%s = table(NaN, NaN, 'VariableNames', ['Frequency', 'SpectrumData']);", feature_ps));  
            end
    
            % Append computed results to the frame table.
            frame = [frame, ...
                table({feature_ps},'VariableNames',{feature_ps})];
    
            %% SpectrumFeatures
            try
                % Compute spectral features.
                % Get frequency unit conversion factor.
                factor = funitconv('Hz', 'rad/TimeUnit', 'seconds');
                ps = eval([feature_ps '.SpectrumData']);
                w = eval([feature_ps '.Frequency']);
                w = factor*w;
                mask_1 = (w>=factor*0) & (w<=factor*12.5000000000001);
                ps = ps(mask_1);
                w = w(mask_1);
    
                % Compute spectral peaks.
                [peakAmp,peakFreq] = findpeaks(ps,w/factor,'MinPeakHeight',-Inf, ...
                    'MinPeakProminence',0,'MinPeakDistance',0.001,'SortStr','descend','NPeaks',3);
                peakAmp = [peakAmp(:); NaN(3-numel(peakAmp),1)];
                peakFreq = [peakFreq(:); NaN(3-numel(peakFreq),1)];
    
                % Extract individual feature values.
                PeakAmp1 = peakAmp(1);
                PeakAmp2 = peakAmp(2);
                PeakAmp3 = peakAmp(3);
                PeakFreq1 = peakFreq(1);
                PeakFreq2 = peakFreq(2);
                PeakFreq3 = peakFreq(3);
                BandPower = trapz(w/factor,ps);
    
                % Concatenate signal features.
                featureValues = [PeakAmp1,PeakAmp2,PeakAmp3,PeakFreq1,PeakFreq2,PeakFreq3,BandPower];
    
                % Package computed features into a table.
                featureNames = {'PeakAmp1','PeakAmp2','PeakAmp3','PeakFreq1','PeakFreq2','PeakFreq3','BandPower'};
                feature_ps_spec = sprintf('%s_ps_spec', feature);
                eval(sprintf("%s = array2table(featureValues,'VariableNames',featureNames);", feature_ps_spec));  
            catch
                % Package computed features into a table.
                featureValues = NaN(1,7);
                featureNames = {'PeakAmp1','PeakAmp2','PeakAmp3','PeakFreq1','PeakFreq2','PeakFreq3','BandPower'};
                feature_ps_spec = sprintf('%s_ps_spec', feature);
                eval(sprintf("%s = array2table(featureValues,'VariableNames',featureNames);", feature_ps_spec));  
            end
    
            % Append computed results to the frame table.
            frame = [frame, ...
                    table({eval(feature_ps_spec)},'VariableNames',{feature_ps_spec})];
        end
        %% Concatenate frames.
        frames = [frames;frame]; %#ok<*AGROW>
    end

    % Write all the results for the current member to the ensemble.
    memberResult = table({frames},'VariableNames',"FRM_1");
    writeToLastMemberRead(outputEnsemble,memberResult)
end

% Gather all features into a table.
featureSpec = ["PeakAmp1","PeakAmp2","PeakAmp3","PeakFreq1","PeakFreq2","PeakFreq3","BandPower"];
featureSign = ["ClearanceFactor","CrestFactor","ImpulseFactor","Kurtosis","Mean","PeakValue","RMS","ShapeFactor","Skewness","Std"];
selectedFeatureNames = [];
for f = 1:length(features)
   
    for g = 1:length(featureSpec)
       selectedFeatureNames = [selectedFeatureNames, strcat('FRM_1/',features(f),'_ps_spec/',featureSpec(g))];
    end

    for h = 1:length(featureSign)
       selectedFeatureNames = [selectedFeatureNames, strcat('FRM_1/',features(f),'_sigstats/',featureSign(h))];
    end
end 
featureTable = readFeatureTable(outputEnsemble,"FRM_1",'Features',selectedFeatureNames,'ConditionVariables',outputEnsemble.ConditionVariables,'IncludeMemberID',true);

% Set SelectedVariables to select variables to read from the ensemble.
outputEnsemble.SelectedVariables = unique([outputEnsemble.DataVariables;outputEnsemble.ConditionVariables;outputEnsemble.IndependentVariables],'stable');

% Gather results into a table.
outputTable = readall(outputEnsemble);

