function stateNext = diffDriveModel(state, control, dt)
    % state: [x, y, theta]
    % control: [v, w]
    x = state(1);
    y = state(2);
    theta = state(3);
    
    v = control(1);
    w = control(2);
    
    % Simple Runge-Kutta integration
    x_next = x + v * cos(theta) * dt;
    y_next = y + v * sin(theta) * dt;
    theta_next = theta + w * dt;
    
    stateNext = [x_next; y_next; theta_next];
end