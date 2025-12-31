function ekf = run_ekf_simulation(initialState, controls, measurements, dt, groundTruthState)
    % RUN_EKF_SIMULATION Runs the EKF SLAM loop and visualizes the result
    
    % 1. Setup EKF Object
    ekf = ekfSLAM('State', initialState, ...
        'StateCovariance', diag([0.01, 0.01, 0.001]), ... 
        'StateTransitionFcn', @diffDriveModel, ... 
        'MaxAssociationRange', 4.0); 
    
    % 2. Define Noise Parameters
    ekf.ProcessNoise = diag([0.1^2 0.01^2]); 
    measNoise = [0.2^2 0.02^2];

    % 3. Setup Visualization
    figure; 
    ax = gca; 
    hold(ax, 'on'); 
    axis equal; grid on;
    title('EKF SLAM: Robot Path & Landmark Uncertainty');
    xlabel('X (meters)'); ylabel('Y (meters)');
    
    % Plot Ground Truth 
    if nargin >= 5 && ~isempty(groundTruthState)
        plot(ax, groundTruthState(:,1), groundTruthState(:,2), 'g--', ...
            'LineWidth', 1, 'DisplayName', 'True Path');
    end
    
    % Initialize Plot Handles
    % Robot Path
    hEst = plot(ax, initialState(1), initialState(2), 'r-', 'LineWidth', 1.5, 'DisplayName', 'EKF Path');
    % Robot Position Dot
    hRobot = plot(ax, initialState(1), initialState(2), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Robot');
    % Landmark Centroids
    hLms = plot(ax, 0, 0, 'ks', 'MarkerFaceColor', 'm', 'DisplayName', 'Mapped Cones');
    % Landmark Uncertainty Ellipses (Initialize as empty line)
    hCov = plot(ax, NaN, NaN, 'b-', 'LineWidth', 0.5, 'DisplayName', '3-\sigma Uncertainty');
    
    legend([hEst, hLms, hCov]); 
    
    % Pre-calculate ellipse unit circle for speed
    theta_circle = linspace(0, 2*pi, 20);
    unit_circle = [cos(theta_circle); sin(theta_circle)];

    % 4. Main Loop
    numSteps = size(controls, 1);
    
    for i = 1:numSteps
        % PREDICT 
        predict(ekf, controls(i,:), dt);
        
        % CORRECT 
        observed = measurements{i};
        if ~isempty(observed)
            correct(ekf, observed, measNoise, [10, 20]); % [10, 20] are validation gate thresholds
        end
        
        % VISUALIZE 
        currState = ekf.State;
        fullCov = ekf.StateCovariance;
        
        % Update Robot Path
        set(hEst, 'XData', [hEst.XData, currState(1)], 'YData', [hEst.YData, currState(2)]);
        set(hRobot, 'XData', currState(1), 'YData', currState(2));
        
        % Update Landmarks & Ellipses
        if length(currState) > 3
            % 1. Extract Landmark Positions
            lms = reshape(currState(4:end), 2, [])';
            set(hLms, 'XData', lms(:,1), 'YData', lms(:,2));
            
            % 2. Calculate Ellipses
            % We build one giant array separated by NaNs to plot all ellipses in one go (faster)
            all_ellipses_x = [];
            all_ellipses_y = [];
            
            % Iterate through landmarks (Indices: 4,5 | 6,7 | etc.)
            for k = 1:size(lms, 1)
                idx = 3 + 2*(k-1) + 1; % Start index (4, 6, 8...)
                
                % Extract 2x2 Covariance for this landmark
                Pk = fullCov(idx:idx+1, idx:idx+1);
                
                % Eigen decomposition for ellipse orientation/size
                [V, D] = eig(Pk); 
                
                % Calculate 3-Sigma Radii (99.7% confidence)
                % Radius = 3 * sqrt(eigenvalue)
                % Transform unit circle: Rotation * Scale * UnitCircle
                ell_points = (V * (3 * sqrt(D))) * unit_circle;
                
                % Shift to landmark position
                ell_points = ell_points + lms(k,:)';
                
                % Append to plot arrays with NaN separator
                all_ellipses_x = [all_ellipses_x, ell_points(1,:), NaN];
                all_ellipses_y = [all_ellipses_y, ell_points(2,:), NaN];
            end
            
            % Update the single plot object
            set(hCov, 'XData', all_ellipses_x, 'YData', all_ellipses_y);
        end
        
        drawnow limitrate; 
    end
end
