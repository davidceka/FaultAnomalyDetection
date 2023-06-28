axis_table = table();
for j = 1:height(dataTable)
    axis_table.acc_x_init_var(j) = dataTable{j, "acc_x_timetable"}{1}.Var1(2)-dataTable{j, "acc_x_timetable"}{1}.Var1(1);
    axis_table.mean_acc_x(j) = mean(dataTable{j, "acc_x_timetable"}{1}.Var1, 'omitnan');
    axis_table.std_acc_x(j) = std(dataTable{j, "acc_x_timetable"}{1}.Var1, 'omitnan');
    
    axis_table.acc_y_init_var(j) = dataTable{j, "acc_y_timetable"}{1}.Var1(2)-dataTable{j, "acc_y_timetable"}{1}.Var1(1);
    axis_table.mean_acc_y(j) = mean(dataTable{j, "acc_y_timetable"}{1}.Var1, 'omitnan');
    axis_table.std_acc_y(j) = std(dataTable{j, "acc_y_timetable"}{1}.Var1, 'omitnan');
    
    axis_table.mean_acc_z(j) = mean(dataTable{j, "acc_z_timetable"}{1}.Var1, 'omitnan');
    axis_table.acc_z_init_var(j) = dataTable{j, "acc_z_timetable"}{1}.Var1(2)-dataTable{j, "acc_z_timetable"}{1}.Var1(1);
    axis_table.std_acc_z(j) = std(dataTable{j, "acc_z_timetable"}{1}.Var1, 'omitnan');

    axis_table.gps_x_init_var(j) = dataTable{j, "gps_x_timetable"}{1}.Var1(2)-dataTable{j, "gps_x_timetable"}{1}.Var1(1);
    axis_table.gps_x_total(j) = calculate_total_distance(dataTable{j, "gps_x_timetable"}{1});
    
    axis_table.gps_y_init_var(j) = dataTable{j, "gps_y_timetable"}{1}.Var1(2)-dataTable{j, "gps_y_timetable"}{1}.Var1(1);
    axis_table.gps_y_total(j) = calculate_total_distance(dataTable{j, "gps_y_timetable"}{1});
    
    axis_table.gps_z_init_var(j) = dataTable{j, "gps_z_timetable"}{1}.Var1(2)-dataTable{j, "gps_z_timetable"}{1}.Var1(1);
    axis_table.gps_z_total(j) = calculate_total_distance(dataTable{j, "gps_z_timetable"}{1});
end
function total_dist = calculate_total_distance(tt)
dist = 0;
prev = tt.Var1(1);
for i = 2:size(tt.Var1,1)
    curr = tt.Var1(i);
    dist = dist + abs(curr - prev);
    prev = curr;
end

% Assegnazione del risultato alla variabile di output
total_dist = dist;
end