function motionplanner(Tsc_initial,Tsc_goal,config_i, config_ref, K_p, K_i)

% Inputs:
%     Tsc_initial - initial configuration of the cube in world coordinates
%     Tsc_goal - desired final configuration of the cube in world coordinates
%     config_i - initial configuration of the youbot
%     config_ref - reference initial configuration of the youbot
%     K_p - proportional gain
%     K_i - integral gain
% 
% Outputs:
%     youbotmotion.csv - a csv file to play the V-REP scene
%     X_err.mat - a data file containing the 6-vector end-effector error
        
        % Youbot geometric parameters
        
        T_b0 = [1 0 0 0.1662;0 1 0 0;0 0 1 0.0026;0 0 0 1];
        
        M_0e = [1 0 0 0.033;0 1 0 0;0 0 1 0.6546;0 0 0 1];
        
        Blist = [0 0 1 0 0.033 0;0 -1 0 -0.5076 0 0;0 -1 0 -0.3526 0 0;0 -1 0 -0.2176 0 0;0 0 1 0 0 0]';

        % initial reference configuration of the end effector
        Tse_initial = config_ref;
        % the number of reference configurations per 0.01 seconds
        k = 1;
        % the end effector's configuration while grasping the cube
        theta = pi/2;
        Tce_grasp = [cos(theta) 0 sin(theta) 0;0 1 0 0;-sin(theta) 0 cos(theta) 0;0 0 0 1];

        % the end effector's standoff configuration above the cube before and after grasping
        Tce_standoff =  [cos(theta) 0 sin(theta) 0;0 1 0 0;-sin(theta) 0 cos(theta) 0.2;0 0 0 1];

        
        
        % generate robot reference trajectory
        [RefTraj,gripper_state] = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_goal, Tce_grasp, Tce_standoff, k);
        
        % timestep
        dt = 0.01;
        
        % vector of velocity limits of each joint and wheel
        vlimit = [35; 35; 35; 35;35; 35; 35; 35; 35];
        
        % pre-allocate space for state vector
        state = zeros(13,length(RefTraj)*k);
        state(:,1) = config_i;
        
        % pre-allocate space for error vector
        X_err = zeros(6,length(RefTraj)*k);
        
        % define psuedoinverse of kinematic model
        l = 0.47/2; 
        r = 0.0475; 
        w = 0.3/2;
        
        F = r/4*[-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w);1 1 1 1;-1 1 -1 1];
        
        % loop through reference configurations
        for i = 1:length(RefTraj)-1
        
            % calculate pose of T_se
            T_sb = [cos(state(1,i)) -sin(state(1,i)) 0 state(2,i);
                sin(state(1,i)) cos(state(1,i)) 0 state(3,i);
                0 0 1 0.0963;
                0 0 0 1];
            T_se_current = T_sb*T_b0*FKinBody(M_0e, Blist, state(4:8,i));
            
            % calculate positioner error for writing
            X_err(:,i) = se3ToVec(MatrixLog6(TransInv(T_se_current)...
                *RefTraj(:,:,i)));
            
            % run feedback control calculation    
            Ve = FeedbackControlnew(T_se_current, RefTraj(:,:,i), ...
                RefTraj(:,:,i+1), K_p, K_i, dt);
            
            % calculate J_arm
            J_arm = JacobianBody(Blist, state(4:8,i)); 

            % calculate J_base
            J_base = Adjoint(TransInv(T_se_current)*T_sb)...
                *[zeros(2,4);F;zeros(1,4)];

            % combine into J_e
            J_e = [J_base J_arm];

            % update state
            controls = pinv(J_e)*Ve;
            state(1:12,i+1) = NextState(state(1:12,i), ...
                [controls(5:end); controls(1:4)], dt, vlimit);
            
            % close gripper if close command is received
            if gripper_state(i) == 1
                state(13,i) = 1;
            end
            
        end
        
        
        % save motion to csv file
        fprintf('Generating csv file\n');
        csvwrite('youbotmotion.csv',state');

        
        %plot the graph
        fprintf('Generating graph\n');
        plot(0:dt:(length(RefTraj)-1)*k*dt, X_err'); 
        title('Xerr Components vs. Time');
        xlabel('Time(s)'); ylabel('Xerr');
        legend('Xerr(1)','Xerr(2)','Xerr(3)','Xerr(4)','Xerr(5)','Xerr(6)')
        
        % save error vector data
        fprintf('Writing error plot data\n');
        save('X_err.mat', 'X_err');
        
        fprintf('Done\n');
end