%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Description: A basic data loading and ground truth plot
%   Last modified: 2023-09-06
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc                         % clear command window
clear all                   % clear workspace
close all                   % close all open figures
%% Load the CSV file
filename = '../../controllers/supervisor/data/ground_truth.csv';
truth_data = readtable(filename);

% Strip spaces from column names
truth_data.Properties.VariableNames = strtrim(truth_data.Properties.VariableNames);

%% Plot the columns
figure;

subplot(2, 2, 1);
plot(truth_data.x, truth_data.y);
title('Position');
xlabel('x [m]');
ylabel('y [m]');
grid on;

subplot(2, 2, 2);
plot(truth_data.time, truth_data.heading);
title('Heading');
xlabel('time [s]');
ylabel('[rad]');
grid on;

subplot(2, 2, 3);
plot(truth_data.time, truth_data.vel_x);
hold on;
plot(truth_data.time, truth_data.vel_y);
title('Velocity');
xlabel('time [s]');
ylabel('[m/s]');
legend({'$\dot{x}$', '$\dot{y}$'}, 'Interpreter', 'latex');
grid on;

subplot(2, 2, 4);
plot(truth_data.time, truth_data.vel_heading);
title('Angular velocity');
xlabel('time [s]');
ylabel('[rad/s]');
grid on;

sgtitle('Pose Ground Truth');
