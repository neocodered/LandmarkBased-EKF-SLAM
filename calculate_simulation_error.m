function errors = calculate_simulation_error(ekf, groundTruthState, mapGroundTruth)
    % CALCULATE_SIMULATION_ERROR Computes robot and map accuracy
    %
    % Inputs:
    %   ekf              - The finished EKF object
    %   groundTruthState - The N x 3 matrix of true robot poses
    %   mapGroundTruth   - The M x 2 matrix of true cone locations
    
    % 1. Calculate Robot Final Position Error 
    % Get the very last estimated state [x, y]
    estPos = ekf.State(1:2)'; 
    % Get the very last true state [x, y]
    truePos = groundTruthState(end, 1:2);
    
    % Euclidean distance
    robotError = norm(estPos - truePos);
    
    % 2. Calculate Map Alignment Error (RMSE)
    % Extract all estimated landmarks from state vector
    if length(ekf.State) > 3
        estLandmarks = reshape(ekf.State(4:end), 2, [])';
        numEst = size(estLandmarks, 1);
        
        squared_error_sum = 0;
        
        % For every estimated landmark, find the closest true cone
        for i = 1:numEst
            current_est = estLandmarks(i, :);
            
            % Calculate distances to ALL true cones
            diffs = mapGroundTruth - current_est;
            distances = sqrt(sum(diffs.^2, 2));
            
            % The error is the distance to the CLOSEST true cone
            min_dist = min(distances);
            squared_error_sum = squared_error_sum + min_dist^2;
        end
        
        landmarkRMSE = sqrt(squared_error_sum / numEst);
    else
        landmarkRMSE = NaN; % No landmarks mapped
    end
    
    % 3. Display Results 
    fprintf('\n==========================================\n');
    fprintf('       EKF SIMULATION ERROR METRICS       \n');
    fprintf('==========================================\n');
    fprintf('Final Robot Position Error:  %.4f meters\n', robotError);
    fprintf('Map Alignment RMSE:          %.4f meters\n', landmarkRMSE);
    
    % Return data struct if needed
    errors.robot_pos = robotError;
    errors.landmark_rmse = landmarkRMSE;
end