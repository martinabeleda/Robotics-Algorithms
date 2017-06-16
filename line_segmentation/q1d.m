% MTRX5700 - Assignment 2
% Martin Abeleda
% Q1 d
% Add your function to one of the above scripts and display the resulting
% lines as part of the laserShow script.
clear all
close all
clc

% Input parameters
threshold = 0.1;    % Corner detection threshold (m)

% load laser files
laser_scans = load('..\datasets\captureScanshornet.txt');
t0 = laser_scans(1,1);

figure('Color',[1 1 1]);
for i = 1:length(laser_scans)
  
     tlaser = laser_scans(i,1) - t0;
     xpoint = zeros(1);
     ypoint = zeros(1);
     for j = 2:size(laser_scans,2)
         range = laser_scans(i,j) / 1000;
         bearing = ((j-1)/2 - 90)*pi/180;
         if (range < 75)
             xpoint = [xpoint range*cos(bearing)];
             ypoint = [ypoint range*sin(bearing)];
         end
     end
     
     corners = lineSegWrapper(xpoint, ypoint, threshold);
     plot(xpoint(:), ypoint(:), '.', corners(:, 1), corners(:, 2));
     axis equal;
     axis([0 10 -5 5]);
     xlabel('X (meter)')
     ylabel('Y (meter)')
     title(sprintf('ACFR indoor SICK data: scan %d',i))
     drawnow
end

% hold on
% corners = lineSegWrapper(xpoint, ypoint, threshold);
% plot(corners(:, 1), corners(:, 2));
% title(sprintf('Edge detection on ACFR SICK data - Threshold: %2.3f', threshold));
