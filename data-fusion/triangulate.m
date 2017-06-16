function [x_laser, y_laser, psi_laser] = triangulate(x_robot, y_robot, psi_robot, ...
    laser_count, beacon_thresh, est_thresh)
%TRIANGULATE Uses laser returns from beacons to triangulate the position of
% the robot. 
%
%   Author: Martin Abeleda
%   Date: 3/5/2017
%   Description:
%    This function takes in a prediction of the robots position and
% bearing, the current laser scan index and the laser beacon threshold. This
% function first looks through the scans to combine consecutive returns into
% a single return. Then these returns are converted into beacon locations.
% The laser threshold is used to compare these beacons with the known
% original locations. If the difference is less than the threshold, then the
% original beacon location is used with that range and bearing reading from
% the laser scan to triangulate the robot location.
    global laser_pos laser_obs laser
    
    % Struct to store beacon metadata
    beacons = struct('tx', 0, 'ty', 0, 'range', 0, 'theta', 0);
    
    % Beacon index
    j = 1;

    % Loop through scans in one row of laser_obs
    % If there are consecutive beacons in the scan, replace them with a 
    % single beacon at the centre
    for i = 1:(size(laser_obs,2)-2)/2

        if laser_obs(laser_count, 2+2*i) == 1
            k = 1; % Count the number of consecutive beacons
            while laser_obs(laser_count, 2+2*(i+k)) == 1

                laser_obs(laser_count, 2+2*(i+k-1)) = 0;
                k = k+1; 

                if 2+2*(i+k) >= size(laser_obs,2)          
                    laser_obs(laser_count, 2+2*(i+k)) = 0;
                    k = k-1;
                    break
                end 
            end

            laser_obs(laser_count, 2+2*(i+k-1)) = 0;
            laser_obs(laser_count, 2+2*(i+ceil((k-1)/2))) = 1;

        end

    end

    % Loop through scans in one row of laser_obs
    for i = 1:(size(laser_obs,2)-2)/2

        if laser_obs(laser_count, 2+2*i) == 1       % Beacon detected

            range = laser_obs(laser_count, 1+2*i);

            if range < 8

                % Coordinates and bearing of beacon in world frame of reference
                theta = ((i-1)/2 - 90)*pi/180;
                tx = range*cos(theta)*cos(psi_robot)+x_robot;
                ty = range*sin(theta)*cos(psi_robot)+y_robot;

                % Preallocate vector to store distance between found beacon and
                % known beacon locations
                dist = zeros(1, size(laser_pos, 1));

                % Compare the obserevation to each of the known locations
                for k = 1:size(laser_pos, 1)

                    dist(k) = sqrt((laser_pos(k,1)-tx)^2 + (laser_pos(k,2)-ty)^2);

                end

                [min_dist, ind] = min(dist);

                % Check if this beacon correlates to one of the known locations
                % Correlation condition is given by thresh. Store the real
                % known location of the beacon
                if min_dist < beacon_thresh

                    beacons(j).range = range;
                    beacons(j).theta = theta;
                    beacons(j).tx = laser_pos(ind,1);
                    beacons(j).ty = laser_pos(ind,2);

                    j = j + 1;  
                end
            end
        end
    end

    % If there are enough beacons, perform triangulation
    if length(beacons) > 2

        % Find all the combinations of 2 beacons
        % Combos is a matrix of the indexes to use together
        combos = nchoosek(1:length(beacons), 2);

        % Preallocate matrix to store running values
        x_obs = zeros(1, size(combos,1));
        y_obs = zeros(1, size(combos,1));
        psi_obs = zeros(1, size(combos,1));
        diff = zeros(1, size(combos,1));

        % Loop through each of the combinations
        % Perform triangulation for each combination of 2 beacons
        for i = 1:size(combos,1)

            b1 = combos(i, 1);
            b2 = combos(i, 2);

            psi_obs(i) = atan2((beacons(b2).ty - beacons(b1).ty),(beacons(b2).tx - beacons(b1).tx)) ...
                - atan2((beacons(b2).range*sin(beacons(b2).theta) - beacons(b1).range*sin(beacons(b1).theta)) ...
                , (beacons(b2).range*cos(beacons(b2).theta) - beacons(b1).range*cos(beacons(b1).theta)));
            x_obs(i) = beacons(b1).tx - beacons(b1).range*cos(beacons(b1).theta + psi_obs(i));
            y_obs(i) = beacons(b1).ty - beacons(b1).range*sin(beacons(b1).theta + psi_obs(i));
            
            % Calculate the distance between prediction and observation
            diff(i) = sqrt((x_obs(i)-x_robot)^2+(y_obs(i)-y_robot)^2);
            
        end
        
        % Filter out the results with diff > thresh
        % Return the average of the results
        x_laser = mean(x_obs(diff <= est_thresh), 'omitnan');
        y_laser = mean(y_obs(diff <= est_thresh), 'omitnan');
        psi_laser = mean(psi_obs(diff <= est_thresh), 'omitnan');
%         
%         % Use estimate with minimum distance from the prediction
%         [~, diff_ind] = min(diff);
        
%         [~,idx,~] = deleteoutliers(diff, 0.05);
%         
%         for i = 1:size(idx,2)
%                 psi_obs(idx(i)) = NaN;
%                 x_obs(idx(i)) = NaN;
%                 y_obs(idx(i)) = NaN;
%         end
%         
%         x_laser = median(x_obs, 'omitnan');
%         y_laser = median(y_obs, 'omitnan');
%         psi_laser = median(psi_obs, 'omitnan');
        
        laser(laser_count).x = x_laser;
        laser(laser_count).y = y_laser;
        laser(laser_count).psi = psi_laser;
        laser(laser_count).beacons = length(beacons);
        
        plot(x_laser, y_laser, 'xg');
        
    else    % If there are not enough beacons

        % Return the initial prediction
        x_laser = x_robot;
        y_laser = y_robot;
        psi_laser = psi_robot;

    end
end

