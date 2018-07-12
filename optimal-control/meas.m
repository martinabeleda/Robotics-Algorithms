function y = meas(x, C, noise) 
% Define noise
v = [normrnd(0, noise(1))
     normrnd(0, noise(2))
     normrnd(0, noise(3))
     normrnd(0, noise(4))];

% Define measurements
y = C*x + v; 
