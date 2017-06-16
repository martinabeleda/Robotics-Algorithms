% MTRX5700 - Assignment 3
% Q1.3 - Data fusion of encoders, GPS, compass and laser
% Martin Abeleda
clear 
close all
clc

% Input Parameters
alpha_gps = 0.1;        % GPS confidence
alpha_comp = 0.5;       % Compass confidence
alpha_las = 0.1;        % Laser confidence
beacon_thresh = 100;    % Laser beacon threshold
est_thresh = 0.5;       % Laser estimation threshold

%% Load Data
% Declare laser data as global variables
global laser_pos laser_obs

% Velocity observations
% time(s) time(micro-seconds) Vel(m/s) Turn rate(rad/s)
vel_obs = load('Assignment_3_Navigation_and_Mapping_Data\velocityObs.txt');
vel = vel_obs(:,3);
psi_dot = vel_obs(:,4);

% GPS observations
% time(s) time(micro-seconds) X(m) Y(m)
pos_obs = load('Assignment_3_Navigation_and_Mapping_Data\positionObs.txt');
x_gps = pos_obs(:, 3);
y_gps = pos_obs(:, 4);

% Compass observations
% time(s) time(micro-seconds) heading(rad)
comp_obs = load('Assignment_3_Navigation_and_Mapping_Data\compassObs.txt');
compass = unwrap(comp_obs(:,3));

% Laser feature positions
% X(m) Y(m)
laser_pos = load('Assignment_3_Navigation_and_Mapping_Data\laserFeatures.txt');

% Laser observations
% time(s) time(micro-seconds) range_1(m - 8.0 max) intensity_1(0/1) ...
laser_obs = load('Assignment_3_Navigation_and_Mapping_Data\laserObs.txt');

%% Time 

% Aggregate the time columns of each dataset
pos_times = pos_obs(:,1) + pos_obs(:,2).*(10^-6);
comp_times = comp_obs(:,1) + comp_obs(:,2).*(10^-6);
vel_times = vel_obs(:,1) + vel_obs(:,2).*(10^-6);
laser_times = laser_obs(:,1) + laser_obs(:,2).*(10^-6);

% Total number of observations
total_observations = length(vel_times) + length(pos_times) + ...
    length(comp_times) + length(laser_times);
t1 = min([pos_times(1), comp_times(1), vel_times(1), laser_times(1)]);

%% Initialisation
global laser

% Initialise counters
pos_count = 1;
comp_count = 1;
vel_count = 1; 
laser_count = 1;

% Current state of robot
state = struct('t', t1, 'x', 0, 'y', 0, 'v', 0, 'psi', 0, 'psidot', 0, 'source', 0);

% Store predictions
predicted = struct('t', t1, 'x', 0, 'y', 0, 'psi', 0);

% Laser triangulated observations
laser = struct('x', 0, 'y', 0, 'psi', 0, 'beacons', 0);

%% Observe and Plot

% figure('Color',[1 1 1]);
% plot(laser_pos(:,1), laser_pos(:,2), 'xr');
% title(sprintf('Robot Path - Data Fusion\n alpha_{gps} = %0.1f, alpha_{comp} = %0.1f, alpha_{laser} = %0.1f', ...
%     alpha_gps, alpha_comp, alpha_las));
% xlabel('x (m)');
% ylabel('y (m)')
% hold on;
% axis equal;

for i = 2:total_observations-50
    
    % Find which dataset is next to update
    [time, source] = min([pos_times(pos_count), comp_times(comp_count), ...
        vel_times(vel_count), laser_times(laser_count)]);
    
    % Copy old state
    state(i) = state(i-1);
    state(i).t = time;
    state(i).source = source;
    delta_t = state(i).t - state(i-1).t;
    
    % Make prediction
    predicted(i).psi = state(i-1).psi + delta_t*state(i-1).psidot;
    predicted(i).x = state(i-1).x + delta_t*state(i-1).v*cos(state(i-1).psi);
    predicted(i).y = state(i-1).y + delta_t*state(i-1).v*sin(state(i-1).psi);
    
    switch source
        case 1  % Position data
            
            state(i).psi = predicted(i).psi;
            state(i).x = (1 - alpha_gps)*predicted(i).x + alpha_gps*x_gps(pos_count);
            state(i).y = (1 - alpha_gps)*predicted(i).y + alpha_gps*y_gps(pos_count);
            
            pos_count = pos_count+1;
            
            plot(x_gps(pos_count), y_gps(pos_count), '.k');
            
        case 2  % Compass data 
            
            state(i).psi = predicted(i).psi + ... 
                alpha_comp*(wrapToPi(compass(comp_count) - predicted(i).psi));   
            state(i).x = predicted(i).x;
            state(i).y = predicted(i).y;
            
            comp_count = comp_count+1;
            
        case 3  % Velocity data
            
            state(i).v = vel(vel_count);
            state(i).psidot = psi_dot(vel_count);
            state(i).psi = predicted(i).psi;
            state(i).x = predicted(i).x;
            state(i).y = predicted(i).y;
            
            vel_count = vel_count+1;
            
        case 4 % Laser data
            
            [x_laser, y_laser, psi_laser] = triangulate(predicted(i).x, ...
                predicted(i).y, predicted(i).psi, laser_count, beacon_thresh, est_thresh);
            
            state(i).psi = (1 - alpha_las)*predicted(i).psi + alpha_las*psi_laser; 
            state(i).x = (1 - alpha_las)*predicted(i).x + alpha_las*x_laser;
            state(i).y = (1 - alpha_las)*predicted(i).y + alpha_las*y_laser;
            
            laser_count = laser_count+1;
            
    end
    
%     plot([state.x], [state.y], '-b', 'LineWidth', 1.5);
%     drawnow
    
end

%% Plot Robot Path
figure('Color',[1 1 1]);
plot(laser_pos(:,1), laser_pos(:,2), 'xr', ...
    [predicted.x], [predicted.y], '-k', ...
    [state.x], [state.y], '-b', ...
    x_gps, y_gps, '.k', ...
    [laser.x], [laser.y], 'xg');
legend('Beacons', 'Predictions', 'Updated', 'GPS observations', 'Laser observations');
title(sprintf('Robot Path - Data Fusion\n alpha_{gps} = %0.1f, alpha_{comp} = %0.1f, alpha_{laser} = %0.1f', ...
    alpha_gps, alpha_comp, alpha_las));
xlabel('x (m)');
ylabel('y (m)');

axis equal;

%% Plot Turn Angle
% figure('Color',[1 1 1]);
% plot([state.t], [state.psi], '-b', ...
%     comp_times, compass, '-r');
% legend('Updated Turn Angle', 'Compass Observations');
% title(sprintf('Turn angle - Data Fusion\n alpha_{gps} = %0.1f, alpha_{comp} = %0.1f, alpha_{laser} = %0.1f', ...
%     alpha_gps, alpha_comp, alpha_las));
% xlabel('Time (s)');
% ylabel('Turn Angle (rad)');











