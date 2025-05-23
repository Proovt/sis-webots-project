%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Description: A basic data loading and ground truth plot
%   Last modified: 2023-09-06
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc                         % clear command window
clear all                   % clear workspace
close all                   % close all open figures
%% Load the CSV file
filename = '../../../controllers/controller/data/example.csv';
truth_name = '../../../controllers/supervisor/data/ground_truth.csv';
acc_filename = '../../../controllers/controller/data/odo_acc.csv';
data = readtable(filename);
truth_data = readtable(truth_name);
acc_data = readtable(acc_filename);
% Strip spaces from column names
data.Properties.VariableNames = strtrim(data.Properties.VariableNames);
acc_data.Properties.VariableNames = strtrim(acc_data.Properties.VariableNames);


%% Plot the odometry x computed using the accelerometer on Webots

% Plot the odometry computed using the accelerometer
f = figure('Name','Webots : Odometry using accelerometer [m/s^2]');
% Plot x : odometry vs ground truth (gps)
plot(truth_data.time, truth_data.x); hold on;
plot(acc_data.time, acc_data.x);
title("x trajectory : odometry vs ground truth (gps)");
legend("Ground Thruth : GPS", "Odometry : Accelerometer");
%ylim([-0.5, 0.5])
xlabel('Time [s]'); ylabel('x [m]');

%y_lim = [min([data.odo_acc_x;  data.pose_x]),max([data.odo_acc_x;  data.pose_x])];
%xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

%% Plot position using accelerometer

f = figure('Name','Webots : Odometry using accelerometer[m/s^2]');

% Plot x : odometry vs ground truth (gps)
plot(truth_data.x, truth_data.y); hold on;
plot(acc_data.x, acc_data.y);
title("x trajectory : odometry vs ground truth (gps)");
legend("Ground Thruth : GPS", "Odometry : Accelerometer");
xlabel('x [m]'); ylabel('y [m]');

%% Plot the odometry heading computed using the accelerometer on Webots

% ROBOT acc mean : -0.00148328 -0.00236683 9.49705

% Plot the odometry computed using the accelerometer
f = figure('Name','Webots : Odometry using accelerometer [m/s^2]');
hold on;
plot(truth_data.time, truth_data.heading);
plot(acc_data.time, acc_data.heading);
legend("Ground Thruth : Heading", "Odometry : Accelerometer");
xlabel('Time [s]'); ylabel('[rad]');
hold off;


%% Plot position using accelerometer

% ROBOT acc mean : -0.00148328 -0.00236683 9.49705

% Plot the odometry computed using the accelerometer
f = figure('Name','Webots : Odometry using accelerometer [m/s^2]');

% Plot x : odometry vs ground truth (gps)
plot(truth_data.x, truth_data.y); hold on;
plot(acc_data.x, acc_data.y);
title("x trajectory : odometry vs ground truth (gps)");
legend("Ground Thruth : GPS", "Odometry : Accelerometer");
xlabel('Time [s]'); ylabel('x [m]');

%y_lim = [min([data.odo_acc_x;  data.pose_x]),max([data.odo_acc_x;  data.pose_x])];
%xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

%% Plot acceleration of accelerometer in x

% Plot the odometry computed using the accelerometer
f = figure('Name','Webots : Odometry using accelerometer [m/s^2]');

R = 0.11;

%truth_acceleration = truth_data.vel_heading.^2/R;

% Plot x : odometry vs ground truth (gps)
plot(truth_data.time, truth_data.heading); hold on;
plot(acc_data.time, acc_data.acc_wx);
plot(acc_data.time, acc_data.acc_wy, DisplayName="Acceleration y");
title("x trajectory : odometry vs ground truth (gps)");
legend("Ground Thruth : GPS", "Odometry : Accelerometer");
xlabel('Time [s]'); ylabel('Acceleration [m/s^2]');

%y_lim = [min([data.odo_acc_x;  data.pose_x]),max([data.odo_acc_x;  data.pose_x])];
%xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

%%
plot(abs(fft(acc_data.acc_wx)/length(acc_data.acc_wx)))