%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc                         % clear command window
clear all                   % clear workspace
close all                   % close all open figures
%% Load the CSV file
odo_filename = '../../controllers/controller/data/odo_acc.csv';
odo_data = readtable(odo_filename);

% Strip spaces from column names
odo_data.Properties.VariableNames = strtrim(odo_data.Properties.VariableNames);

%%
figure
plot(odo_data.time, odo_data.accx, DisplayName="Acc")

figure
plot(odo_data.time, odo_data.velx, DisplayName="Vel")

figure
plot(odo_data.time, odo_data.x, DisplayName="Pos")


