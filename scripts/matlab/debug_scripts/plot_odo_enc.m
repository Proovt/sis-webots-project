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
enc_filename = '../../../controllers/controller/data/odo_enc.csv';
data = readtable(filename);
truth_data = readtable(truth_name);
enc_data = readtable(enc_filename);
% Strip spaces from column names
data.Properties.VariableNames = strtrim(data.Properties.VariableNames);
enc_data.Properties.VariableNames = strtrim(enc_data.Properties.VariableNames);


%% Plot the odometry computed using the accelerometer on Webots

% ROBOT acc mean : -0.00148328 -0.00236683 9.49705

% Plot the odometry computed using the accelerometer
f = figure('Name','Webots : Odometry using accelerometer [m/s^2]');

% Plot x : odometry vs ground truth (gps)
plot(truth_data.time, truth_data.x); hold on;
plot(enc_data.time, enc_data.x);
title("x trajectory : odometry vs ground truth (gps)");
legend("Ground Thruth : GPS", "Odometry : Wheel encoder");
xlabel('Time [s]'); ylabel('x [m]');

%y_lim = [min([data.odo_acc_x;  data.pose_x]),max([data.odo_acc_x;  data.pose_x])];
%xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));



%% Plot position using encoder

% ROBOT acc mean : -0.00148328 -0.00236683 9.49705

% Plot the odometry computed using the accelerometer
f = figure('Name','Webots : Odometry using wheel encoder [m/s^2]');

% Plot x : odometry vs ground truth (gps)
plot(truth_data.x, truth_data.y); hold on;
plot(enc_data.x, enc_data.y);
title("x trajectory : odometry vs ground truth (gps)");
legend("Ground Thruth : GPS", "Odometry : Accelerometer");
xlabel('x [m]'); ylabel('y [m]');

%y_lim = [min([data.odo_acc_x;  data.pose_x]),max([data.odo_acc_x;  data.pose_x])];
%xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

%% Plot acceleration of accelerometer in x

% Plot the odometry computed using the accelerometer
f = figure('Name','Webots : Odometry using accelerometer [m/s^2]');

% Plot x : odometry vs ground truth (gps)
plot(truth_data.time, truth_data.vel_heading); hold on;
plot(enc_data.time, enc_data.speed_wx);
title("x trajectory : odometry vs ground truth (gps)");
legend("Ground Thruth : GPS", "Odometry : Accelerometer");
xlabel('time [s]'); ylabel('speed [m/s]');

%y_lim = [min([data.odo_acc_x;  data.pose_x]),max([data.odo_acc_x;  data.pose_x])];
%xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

%% Plot the odometry heading computed using the accelerometer on Webots

% ROBOT acc mean : -0.00148328 -0.00236683 9.49705

% Plot the odometry computed using the accelerometer
f = figure('Name','Webots : Odometry using wheel encoders [m/s^2]');
hold on;
plot(truth_data.time, truth_data.heading);
plot(enc_data.time, enc_data.heading);
legend("Ground Thruth : Heading", "Odometry : Heading");
xlabel('Time [s]'); ylabel('[rad]');
hold off;