%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Description: A basic data loading and ground truth plot
%   Last modified: 2023-09-06
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc                         % clear command window
clear all                   % clear workspace
close all                   % close all open figures
%% Load the CSV file
node_name = '../../../controllers/controller/data/sensor_node.csv';
truth_name = '../../../controllers/supervisor/data/ground_truth.csv';

node_data = readtable(node_name);
truth_data = readtable(truth_name);

% Strip spaces from column names
node_data.Properties.VariableNames = strtrim(node_data.Properties.VariableNames);

%%
hold on
plot(truth_data.x(sync_time_idxs), truth_data.y(sync_time_idxs), node_data.signal_strength);
scatter3(node_data.x(1), node_data.y(1), 0)
xlabel('x [m]')
ylabel('y [m]')
zlabel('Signal Strength')

hold off
%%
plot(node_data.time, node_data.signal_strength)