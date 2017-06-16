% MTRX5700 - Assignment 2
% Martin Abeleda
% Q1 e test scaffold
clear all
close all
clc

% Input parameters
threshold = 0.1;        % Vertex detection threshold (m)
tolerance = 0.06;       % Corner tolerance
scan = 1;               % Scan selection    

% load laser files
laser_scans = load('..\datasets\captureScanshornet.txt');
t0 = laser_scans(1,1);

figure('Color',[1 1 1]);
for i = 1:scan
  
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
     plot(xpoint(:), ypoint(:), '.');
     axis equal;
     axis([0 10 -5 5]);
     xlabel('X (meter)')
     ylabel('Y (meter)')
     title(sprintf('ACFR indoor SICK data: scan %d',i))
     drawnow
end

hold on
vertices = lineSegWrapper(xpoint, ypoint, threshold);
corners = findCorners(vertices, tolerance);

plot(vertices(:, 1), vertices(:, 2), 'b');
plot(corners(:, 1), corners(:, 2), 'o');
title(sprintf('Corner detection on ACFR SICK data - Tolerance: %2.3f', tolerance));
