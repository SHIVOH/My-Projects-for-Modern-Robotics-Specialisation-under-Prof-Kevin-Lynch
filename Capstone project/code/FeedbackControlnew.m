function Ve = FeedbackControlnew(T_se, RfTraj,RfTraj_next, K_p, K_i, dt)
%{ 
FeedbackControlnew Control the trajectory of the youbot over time
    Inputs:
        T_se - the current actual end effector configuration
        RfTraj - the current end-effector reference cofiguration
       RfTraj_next - the end-effector reference cofiguration at the 
            next time step
        K_p - the P gain matrix
        K_i - the I gain matrix
        dt - timestep
    
    Outputs:
        Ve - the commanded end-effector twist in the end effector frame
%}
    % store X_int as a persistent variable
    persistent X_int;
    if isempty(X_int)
        X_int = zeros(4,4);
    end

    % calculate error
    X_err = MatrixLog6(TransInv(T_se)*RfTraj);

    % update integral error
    X_int = X_int + X_err*dt;

    % extract feedforward reference twist
    Vd = 1/dt*MatrixLog6(TransInv(RfTraj)*RfTraj_next);

    % calculate feedforward plus feedback control law
    Ve = Adjoint(TransInv(T_se)*RfTraj)*se3ToVec(Vd) + K_p*se3ToVec(X_err) ...
        + K_i*se3ToVec(X_int);

end