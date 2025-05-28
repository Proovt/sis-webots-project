%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Description: A basic data loading and ground truth plot
%   Last modified: 2023-09-06
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%clc                         % clear command window
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



sigma_enc_filename = '../../../controllers/controller/data/odo_enc_sigma.csv';
sigma_enc_data = readtable(sigma_enc_filename);
sigma_acc_filename = '../../../controllers/controller/data/odo_acc_sigma.csv';
sigma_acc_data = readtable(sigma_acc_filename);
sigma_filename = '../../../controllers/controller/data/odo_sigma.csv';
sigma_data = readtable(sigma_filename);


sensors = [1.79621 -0.115357;
    4.62 1.21;
    1.30989 2.56938;
    4.06932 3.90368];


radius = 0.46; % Circle radius
%% measure accuracy
idxs = zeros(sum(odo_data.time <= 350), 1);

last_idx = -1;

for i = 1:length(odo_data.x)
    if odo_data.time(i) > 350
        last_idx = i - 1;
        break
    end
    idxs(i) = find(truth_data.time == odo_data.time(i));
end

if last_idx == -1
    last_idx = length(odo_data.x);
end

odo_pos = [odo_data.x(1:last_idx) odo_data.y(1:last_idx)];
true_pos = [truth_data.x(idxs) truth_data.y(idxs)];

error = vecnorm(odo_pos - true_pos, 2, 2);
err = sum(error);
fprintf("Error: %f, Max Distance: %f\n", err, max(error));

%% Plot position using both encoder and accelerometer

% Plot the odometry computed using the accelerometer
f = figure('Name','Webots : Odometry using wheel encoder [m/s^2]');

%scale = 1:length(acc_data.x);
%scale = scale' / 800 + 1;

% Plot x : odometry vs ground truth (gps)
plot(truth_data.x, truth_data.y, DisplayName="Ground Thruth : GPS"); hold on;
plot(odo_data.x, odo_data.y, DisplayName="Fusion", LineWidth=0.1);
plot(enc_data.x, enc_data.y, LineStyle="--", DisplayName="Encoders");
plot(acc_data.x, acc_data.y, LineStyle="--", DisplayName="Accelerometer");
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
ylim([-1 7])
xlim([-1 7])

%% Plot heading comparison

% Plot the odometry computed using the accelerometer
f = figure('Name','Webots : Heading comparison [°]');
hold on
% Plot x : odometry vs ground truth (gps)
plot(truth_data.time, truth_data.heading * 180 / pi, DisplayName="Ground Truth");
plot(odo_data.time, odo_data.heading * 180 / pi, DisplayName="Kalman");
plot(enc_data.time, enc_data.heading * 180 / pi, LineStyle="--", DisplayName="Odometry: Encoder");
plot(acc_data.time, acc_data.heading * 180 / pi, LineStyle="--", DisplayName="Odometry: Accelerometer");

title(" encoder vs accelerometer");
legend
xlabel('time [s]'); ylabel('heading [°]');
hold off

%%

a = input("Current uncertainty: ");
Uncertainty = [Uncertainty; a, error];

%%
Uncertainty = array2table(Uncertainty, "VariableNames", ["Uncertainty", "Error"]);
writetable(Uncertainty, "uncertainty_vs_error.csv");
%% uncertainty
hold on
plot(sigma_data.heading, DisplayName="sigma")
%plot(sigma_enc_data.heading, DisplayName="sigma enc")
plot(sigma_acc_data.heading, DisplayName="sigma acc")
legend
hold off


%%
Uncertainty = [];