%% AMME5520 Assignment 2
% Navigation and control through an obstacle field.
% Author: Martin Abeleda
% Date: 4/6/2018
clear; close all; clc
global Width Height BufferDist nNodes conn dispPRM dispNodes

% Game Variables
Width = 300;    % Width of the configuration space
Height = 150;   % Height of the configuration space
BufferDist = 2; % Minimal distance to obstacles
nNodes = 100;   % Number of nodes in the PRM
conn = 20;      % Connectivity of the PRM
numObst = 10;   % Control the number of obstacles, e.g. 10, 20, 30, 40.
Adim = 10;      % Control the "average" size of the obstacles.

% Display
dispPRM = true;
dispNodes = true;

%% Pre-processing Phase
% Generate configuration space and probabilistic road-map
fprintf('Generating configuration space with %d obstacles\n', numObst);

As = cell(numObst,1);
cs = cell(numObst,1);

figure('Color', [1 1 1]);

dimensions = [0 Width 0 Height];

% Create obstacles only in the middle 60%
leftlim = 0.2*(dimensions(2)-dimensions(1));
rightlim = 0.8*(dimensions(2)-dimensions(1));

for k = 1:numObst
    % Generate ellipse in the form (x-c)'A(x-c)=1
    
     L = randn(2,2);
     As{k} = (0.4*eye(2)+ L'*L)/Adim^2;
     tmp = rand(2,1);
     cs{k} = [leftlim+(rightlim-leftlim)*tmp(1);dimensions(3)+(dimensions(4)-dimensions(3))*tmp(2)];
     Ellipse_plot(As{k},cs{k});
     
end
title(sprintf('Plot of PRM in Configuration Space. PRM: %d nodes and %d connections', nNodes, conn));
xlabel('X (m)');
ylabel('Y (m)');
axis(dimensions);

% Create PRM
G = generatePRM(As, cs);

%% Path Planning
% Code for adding start and finish configurations to the PRM and generating
% the optimal path
% Define starting and ending points
bl = [10; 10]; br = [290; 10];
tl = [10; 140]; tr = [290; 140];
ml = [10; 75]; mr = [290; 75];

% Choose start and end configuration
start = tl;
finish = br;
nodeList = ComputePath(As, cs, G, start, finish, 'g');

%% Tune LQR Parameters
% Model Parameters
global Ih mh rh Ib mb rb g
Ih = 0.0256;  mh = 3; rh = 0.33; Ib = 0.25; mb = 6; rb = 0.25; g = 9.81;

% Compute the controller K using LQR
Q = zeros(10);
Q(1,1) = 100;       % Weighting for x
Q(2,2) = 10;        % Weighting for theta_y
Q(3,3) = 0;         % Weighting for xdot        
Q(6,6) = 100;       % Weighting for y
Q(7,7) = 10;        % Weighting for theta_x
Q(8,8) = 0;         % Weighting for ydot
R = diag([1 1]);    % Control input weighting

% Generate linearised state-space model
[A, B] = LinearisedModel();

% Define measured states
C = [1 0 0 0 0 0 0 0 0 0        % x - triangulation from bearings
     0 1 0 0 0 0 0 0 0 0        % theta_y - encoder measurements
     0 0 0 0 0 1 0 0 0 0        % y - triangulation from bearings
     0 0 0 0 0 0 1 0 0 0];      % theta_x - encoder measurements
     % note - unsure yet how to define measurements for linear acceleration

% Solve the optimal control for the linear model
[K, ~] = lqr(A,B,Q,R);

fprintf('LQR derived gain matrix: K = \n');
disp(K);

%% Initialise Simulation Variables
v = 2;    % Velocity (m/s)  
h = 0.01; % Sample time (s)
ds = v*h; % Distance travelled between samples (m)

BufferDist = 0;

% Convert node list to trajectory of equally spaced targets
trajectory = ComputeTrajectory(nodeList, ds);

% Initial conditions
x0 = zeros(10,1); 
x0(1) = start(1);   % x0
x0(6) = start(2);   % y0

% Declare variables to store the states during simulation
ts = 0;             % Simulation Times (s)
xs = x0;            % Simulation States

% Kalman filter
xhats = x0;         % Initial estimate assume we know the starting point

% Sensor characteristics        
sigma_bearing = 0.5;               % assumption (m) - need to add bearing measurement and triangulation     
sigma_theta = deg2rad(360/2000);    % assume accuracy of not worse than +-1 count   
noise = [sigma_bearing; sigma_theta; sigma_bearing; sigma_theta]; 

% Covariance Matrices     
X = eye(10)*0.0001;         % Initial error covariance matrix
Q = eye(10)*0.00001;        % Process noise covariance - model uncertainty
R = [sigma_bearing^2 0 0 0  % Measurement noise covariance matrix    
     0 sigma_theta^2 0 0
     0 0 sigma_bearing^2 0
     0 0 0 sigma_theta^2];

%% Simulate Closed-loop system
fprintf('Simulating system...\n');
stop = false;
collision = false;
k = 1;
while (stop ~= 1)
       
    % Current state.
    xt = xs(:,k);
    xhat = xhats(:,k);
       
    % Next Desired State
    if k >= length(trajectory)
        xn = [finish(1); 0; 0; 0; 0; finish(2); 0; 0; 0; 0];
    else
        theta = atan2(trajectory(2,k+1) - trajectory(2,k), ...
                      trajectory(1,k+1) - trajectory(1,k));
        v_x = cos(theta)*v;
        v_y = sin(theta)*v;
        xn = [trajectory(1,k+1); 0; v_x; 0; 0; trajectory(2,k+1); 0; v_y; 0; 0];
    end

    % Get current measurement
    yt = meas(xt, C, noise);
    
    % Compute control assuming perfect state feedback
    ut = ComputeControl(K, xt, xn); 
    %ut = ComputeControl(K, xhat, xn); % Compute control based on state estimate
    
    % Use Runge Kutta to compute next state
    xs(:,k+1) = RungeKutta4(@RobotDynamics, xt, ut, 0, h);
    
    % State Estimator
    [xhats(:,k+1), P] = EstimateState(xhat, yt, ut, h, X, C, R, Q, A, B);
    
    ts(k+1) = ts(k)+h;
    us(:,k) = ut;
    ys(:,k) = yt;

    % Stop if goal reached or collision
    if CheckCollisionNode([xt(1) xt(6)]', As, cs)
        fprintf('Robot collided with obstacle at (%f.2, %f.2)\n', xt(1), xt(6));
        collision = true;
        stop = true;
    elseif k >= size(trajectory,2)-2
        fprintf('\nRobot successfully reached the target in %f seconds\n', ts(k));
        stop = true;
    end
    k = k+1;
end

% Generate new figure for simulation
figure('Color', [1 1 1]);
if collision, plot(xt(1), xt(6), 'r*', 'MarkerSize', 10), end
for k = 1:numObst
     Ellipse_plot(As{k},cs{k});   
end
plot(ys(1,:), ys(3,:), 'c-',...
     xs(1,:), xs(6,:), 'r-', ...
     xhats(1,:), xhats(6,:), 'b--');     %trajectory(1,:), trajectory(2,:), 'g--', ...
title('Simulation of robot in Configuration Space');
xlabel('X (m)');
ylabel('Y (m)');
axis(dimensions);

% Plot head angle and motor torque
% figure('Color', [1 1 1]);
% subplot(2, 1, 1);
% plot(ts, xs(2,:), ts, xs(7,:));
% legend('\theta_y', '\theta_x');
% xlabel('Simulation Time (s)');
% ylabel('Angle (rad)');
% 
% subplot(2, 1, 2);
% plot(ts, xs(2,:), ts, xs(7,:));
% legend('\tau_y', '\tau_x');
% xlabel('Simulation Time (s)');
% ylabel('Motor Torue (N.m)');
