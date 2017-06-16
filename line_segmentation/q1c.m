% MTRX5700 - Assignment 2
% Martin Abeleda
% Q1 c
% Implement the line splitting/segmentation algorithm described in the
% lecture
clear all
close all
clc

% Input parameters
threshold = 1.28;    % Corner detection threshold (m)

% Generate a triangular wave
x = 1:0.5:20;
y = sawtooth(x, 0.5);
figure('Color',[1 1 1]);
plot(x, y, '*b');
hold on

% Corner detection
corners = lineSegWrapper(x, y, threshold);
plot(corners(:, 1), corners(:, 2));
title(sprintf('Edge detection on triangle wave - Threshold: %2.3f', threshold));
