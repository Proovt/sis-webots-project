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
odo_filename = '../../../controllers/controller/data/odo.csv';
data = readtable(filename);
truth_data = readtable(truth_name);
odo_data = readtable(odo_filename);
% Strip spaces from column names
data.Properties.VariableNames = strtrim(data.Properties.VariableNames);
odo_data.Properties.VariableNames = strtrim(odo_data.Properties.VariableNames);


acc_filename = '../../../controllers/controller/data/odo_acc.csv';
acc_data = readtable(acc_filename);
acc_data.Properties.VariableNames = strtrim(acc_data.Properties.VariableNames);

enc_filename = '../../../controllers/controller/data/odo_enc.csv';
enc_data = readtable(enc_filename);
enc_data.Properties.VariableNames = strtrim(enc_data.Properties.VariableNames);

sensors = [1.79621 -0.115357;
    4.62 1.21;
    1.30989 2.56938;
    4.06932 3.90368];


radius = 0.46; % Circle radius
%% Plot position using both encoder and accelerometer

% Plot the odometry computed using the accelerometer
f = figure('Name','Webots : Odometry using wheel encoder [m/s^2]');

%scale = 1:length(acc_data.x);
%scale = scale' / 800 + 1;

% Plot x : odometry vs ground truth (gps)
plot(truth_data.x, truth_data.y, DisplayName="Ground Thruth : GPS"); hold on;
scatter(odo_data.x, odo_data.y, DisplayName="Fusion", LineWidth=0.4);
plot(enc_data.x, enc_data.y, LineStyle="--", DisplayName="Encoders");
% plot(acc_data.x ./ scale, acc_data.y ./ scale, LineStyle="--", DisplayName="Accelerometer");
scatter(sensors(:, 1), sensors(:, 2), DisplayName="Sensor", Marker="x")

for i = 1:length(sensors)
    x_point = sensors(i, 1);  % X-coordinate of the point
    y_point = sensors(i, 2);  % Y-coordinate of the point
    
    % Plot a circle around the point (x_point, y_point) with a radius of 0.46
    theta = linspace(0, 2*pi, 100); % Parametric angle from 0 to 2*pi
    x_circle = x_point + radius * cos(theta); % X coordinates of the circle
    y_circle = y_point + radius * sin(theta); % Y coordinates of the circle
    
    % Plot the circle
    plot(x_circle, y_circle, 'r-', 'LineWidth', 1) % You can adjust the color and line width as needed
end

title("x trajectory : odometry vs ground truth (gps)");
legend();
xlabel('x [m]'); ylabel('y [m]');
ylim([-1 6])

%y_lim = [min([data.odo_acc_x;  data.pose_x]),max([data.odo_acc_x;  data.pose_x])];
%xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));
%% measure accuracy
idxs = zeros(sum(odo_data.time <= 350), 1);

for i = 1:length(odo_data.x)
    if odo_data.time(i) > 350
        last_idx = i - 1;
        break
    end
    idxs(i) = find(truth_data.time == odo_data.time(i));
end

odo_pos = [odo_data.x(1:last_idx) odo_data.y(1:last_idx)];
true_pos = [truth_data.x(idxs) truth_data.y(idxs)];

error = norm(odo_pos - true_pos);
sprintf("Error: %f", error)
%%

a = input("Current uncertainty: ");
Uncertainty = [Uncertainty; a, error];

%%
Uncertainty = array2table(Uncertainty, "VariableNames", ["Uncertainty", "Error"]);
writetable(Uncertainty, "uncertainty_vs_error.csv");
%%
Uncertainty = [];
%% Plot the odometry computed using the accelerometer on Webots

% ROBOT acc mean : -0.00148328 -0.00236683 9.49705

% Plot the odometry computed using the accelerometer
f = figure('Name','Webots : Odometry using accelerometer [m/s^2]');

% Plot x : odometry vs ground truth (gps)
plot(truth_data.time, truth_data.x); hold on;
plot(odo_data.time, odo_data.x);
title("x trajectory : odometry vs ground truth (gps)");
legend("Ground Thruth : GPS", "Odometry : Wheel encoder");
xlabel('Time [s]'); ylabel('x [m]');

%y_lim = [min([data.odo_acc_x;  data.pose_x]),max([data.odo_acc_x;  data.pose_x])];
%xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

%% Plot acceleration of accelerometer in x

% Plot the odometry computed using the accelerometer
f = figure('Name','Webots : Odometry using accelerometer [m/s^2]');

% Plot x : odometry vs ground truth (gps)
plot(truth_data.time, truth_data.vel_heading); hold on;
plot(odo_data.time, odo_data.speed_wx);
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
plot(odo_data.time, odo_data.heading);
legend("Ground Thruth : Heading", "Odometry : Heading");
xlabel('Time [s]'); ylabel('[rad]');
hold off;