%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Description: A basic data loading and ground truth plot
%   Last modified: 2023-09-06
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc                         % clear command window
clear all                   % clear workspace
close all                   % close all open figures
%% Load the CSV file
sensor_name = '../../../controllers/controller/data/sensor_data.csv';
truth_name = '../../../controllers/supervisor/data/ground_truth.csv';

sensor_data = readtable(sensor_name);
truth_data = readtable(truth_name);

% Strip spaces from column names
sensor_data.Properties.VariableNames = strtrim(sensor_data.Properties.VariableNames);

%%
sync_time_idxs = zeros(length(sensor_data.time), 1);

for i = 1:length(sensor_data.time)
    sync_time_idxs(i) = find(truth_data.time == sensor_data.time(i));
end

%first_time = sensor_data.time(1);
%truth_data_first_time_idx = find(truth_data.time == first_time);
%sync_time_idxs = truth_data_first_time_idx:4:length(truth_data.time);


%%
hold on
plot3(truth_data.x(sync_time_idxs), truth_data.y(sync_time_idxs), sensor_data.signal_strength);
scatter3(sensor_data.x(1), sensor_data.y(1), 0)
xlabel('x [m]')
ylabel('y [m]')
zlabel('Signal Strength')

hold off
%%
plot(sensor_data.time, sensor_data.signal_strength)