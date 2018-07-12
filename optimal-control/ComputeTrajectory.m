function trajectory = ComputeTrajectory(params, ds)
% Converts waypoints to trajectory of equally spaced targets

waypoints = params{1};
trajectory = waypoints(:,1); % start at the first node

% For each straight line path
for i = 1:size(waypoints,2)-1
    x1 = waypoints(:,i); x2 = waypoints(:,i+1);
    dist = norm(x1-x2); % Euclidean dist
    coef = polyfit([x1(1), x2(1)], [x1(2), x2(2)], 1);
    x_points = linspace(x1(1),x2(1),floor(dist/ds));
    y_points = coef(1)*x_points+coef(2);
    x_points = x_points(2:end); % remove duplicate starting point
    y_points = y_points(2:end); % as above
    trajectory = [trajectory vertcat(x_points,y_points)];
end
end

