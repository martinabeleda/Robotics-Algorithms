function [xhat_next, P_next] = EstimateState(xhat, yt, ut, h, X, C, R, Q, A, B)
% Kalman Filter State Estimator

    % Predict next state 
    xhat_pred = RungeKutta4(@RobotDynamics, xhat, ut, 0, h);
    X_pred = X;
    %xhat_pred = A*xhat + B*ut;  
    %X_pred = A*P*A' + Q;
    
    % Measurement update
    L = (X_pred*C')/(C*X_pred*C' + R);        
    xhat_next = xhat_pred + L*(yt - C*xhat_pred);
    P_next = (eye(10) - L*C)*X_pred; 

end

