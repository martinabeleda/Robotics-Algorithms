% MTRX5700 - Assignment 2
% Martin Abeleda
% Q3 b code
clear 
close all

% Input parameters
step_size = 1;
start = 100;
finish = 150;

% load laser files
laser_scans=load('..\datasets\captureScanshornet.txt');
t0 = laser_scans(1,1);
x_running = 0;
y_running = 0;
theta_running = 0;
x_robot = 0;
y_robot = 0;
robot_path = [0, 0];
figure('Color',[1 1 1]);

for i = start:step_size:finish
    
    clf
    
    xA = zeros(1);
    yA = zeros(1);
    for j = 2:size(laser_scans,2)
        range = laser_scans(i,j) / 1000;
        bearing = ((j-1)/2 - 90)*pi/180;
        if (range < 75)
            xA = [xA range*cos(bearing)];
            yA = [yA range*sin(bearing)];
        end
    end
    
    x = zeros(1);
    y = zeros(1);
    for j = 2:size(laser_scans,2)
        range = laser_scans(i+step_size,j) / 1000;
        bearing = ((j-1)/2 - 90)*pi/180;
        if (range < 75)
            x = [x range*cos(bearing)];
            y = [y range*sin(bearing)];
        end
    end
    
    % Find the translation and rotation between successive scans
    deltaPose = zeros(3,1);
    [deltaPose_bar, deltaPose_bar_Cov, N] = ICPv4(deltaPose, [xA;yA], [x;y]);
    
    % Running total of rotations and translations
    x_running = x_running + deltaPose_bar(1);
    y_running = y_running + deltaPose_bar(2);
    theta_running = theta_running + deltaPose_bar(3);
    
    % Apply rotations and translations to scan points
    x = zeros(1);
    y = zeros(1);
    for j = 2:size(laser_scans,2)
        range = laser_scans(i+step_size,j) / 1000;
        bearing = deg2rad((j-1)/2 - 90) + theta_running;
        if (range < 75)
            x = [x range*cos(bearing)+x_running];
            y = [y range*sin(bearing)+y_running];
        end
    end
    
    robot_path = [robot_path; [x_running y_running]];
    hold on
    plot(x(:), y(:), 'k.');
    plot(robot_path(:,1), robot_path(:,2), 'r.');
    axis equal;
    axis([-10 10 -10 10]);
    xlabel('X (meter)')
    ylabel('Y (meter)')
    title(sprintf('ACFR indoor SICK data: scan %d',i))
    drawnow
end
