function ekf = run_ekf_simulation(initialState, controls, measurements, dt, groundTruthState)
    % RUN_EKF_SIMULATION Runs the EKF SLAM loop and visualizes the result
    %
    % Inputs:
    %   initialState     - [x, y, theta] starting pose
    %   controls         - [v, w] control inputs for every step
    %   measurements     - Cell array of sensor data
    %   dt               - Time step (seconds)
    %   groundTruthState - (Optional) True path for comparison plotting
    
    % 1. Setup EKF Object
    ekf = ekfSLAM('State', initialState, ...
        'StateCovariance', diag([0.01, 0.01, 0.001]), ... % Low initial uncertainty
        'StateTransitionFcn', @diffDriveModel, ... 
        'MaxAssociationRange', 4.0); 

    % 2. Define Noise Parameters
    ekf.ProcessNoise = diag([0.1^2 0.01^2]); % Trust motion model (v, w)
    % OLD: [0.2^2 0.02^2]
    % NEW: Increase range variance significantly (0.5^2)
    measNoise = [0.2^2 0.02^2];

    % 3. Setup Visualization
    figure; 
    ax = gca; 
    hold(ax, 'on'); 
    axis equal; grid on;
    title('EKF SLAM on Track');
    xlabel('X (meters)'); ylabel('Y (meters)');
    
    % Plot Ground Truth 
    if nargin >= 5 && ~isempty(groundTruthState)
        plot(ax, groundTruthState(:,1), groundTruthState(:,2), 'g--', ...
            'LineWidth', 1, 'DisplayName', 'True Path');
    end
    
    % Initialize Plot Handles for Speed
    hEst = plot(ax, initialState(1), initialState(2), 'r-', 'LineWidth', 1.5, 'DisplayName', 'EKF Path');
    hLms = plot(ax, 0, 0, 'ks', 'MarkerFaceColor', 'm', 'DisplayName', 'Mapped Cones');
    %legend([hEst, hLms]); % Update legend
    
    % 4. Main Loop
    numSteps = size(controls, 1);
    
    for i = 1:numSteps
        % PREDICT 
        predict(ekf, controls(i,:), dt);
        
        % CORRECT 
        observed = measurements{i};
        if ~isempty(observed)
            correct(ekf, observed, measNoise, [10, 20]);
        end
        
        % VISUALIZE 
        currState = ekf.State;
        
        % Update Robot Position Dot
        plot(ax, currState(1), currState(2), 'r.');
        
        % Update landmarks only if any exists
        if length(currState) > 3
            lms = reshape(currState(4:end), 2, [])';
            set(hLms, 'XData', lms(:,1), 'YData', lms(:,2));
        end
        
        drawnow limitrate; % Drawing
    end
end