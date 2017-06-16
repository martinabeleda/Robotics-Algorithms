% MTRX5700 - Assignment 2
% Martin Abeleda
% Q3 a code
clear 
close all

thresh = 1e-9;
laser_scans=load('..\datasets\captureScanshornet.txt');
t0 = laser_scans(1,1);

% Scan A
i = 1000;
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

% Scan B
i = 1100;
xB = zeros(1);
yB = zeros(1);
for j = 2:size(laser_scans,2)
 range = laser_scans(i,j) / 1000;
 bearing = ((j-1)/2 - 90)*pi/180;
 if (range < 75)
     xB = [xB range*cos(bearing)];
     yB = [yB range*sin(bearing)];
 end
end

deltaPose_bar = zeros(3,1);
deltaPoses = [0;0;0];
i = 2;
figure('Color',[1 1 1]);
clf
hold on    
plot(xA,yA,'b+')
plot(xB,yB,'r.')
axis equal

while (1) 
    [deltaPose_bar, deltaPose_bar_Cov, N] = ICPv4(deltaPose_bar, [xA;yA], [xB;yB]);
    deltaPoses = [deltaPoses, deltaPose_bar];
    diff = deltaPoses(:,i) - deltaPoses(:,i-1);
    if abs(diff) < thresh
        break
    end
    i = i+1;
    newB = head2tail_no_theta(deltaPose_bar, [xB;yB]);
    new_xB = newB(1,:);
    new_yB = newB(2,:);
end

plot(new_xB,new_yB,'kx')
axis equal
legend('Map','initial guess','ICP')
xlabel('X (meter)')
ylabel('Y (meter)')
title(sprintf('The ICP algorithm, %d iterations',i-1)) 

figure('Color',[1 1 1]);
pose = [deltaPoses; 0:i-1];
plot(pose(4,:),pose(1,:), '-x', pose(4,:), pose(2,:), '-x', pose(4,:), pose(3,:), '-x')
legend('delta_x','delta_y','delta_t')
xlabel('Iterations')
ylabel('Magnitude')
title(sprintf('The ICP algorithm, %d iterations',i-1)) 