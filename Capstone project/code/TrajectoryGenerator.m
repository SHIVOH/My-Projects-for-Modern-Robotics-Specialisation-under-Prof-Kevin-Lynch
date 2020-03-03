function [RefTraj,gripperstate] = TrajectoryGenerator(Tse_initial,...
    Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k)
%{

    Inputs:
        Tse_initial - initial configuration of the end effector in the
            reference trajectory in the space frame
        Tsc_initial - the cube's initial configuration in the space frame
        Tsc_final - the cube's desired final configuration in the space frame
        Tce_grasp - the end effector's configuration when it is grasping
            the cube
        Tce_standoff - the end effector's standoff configuration above the
            cube before and after grasping
        k - the number of reference configurations per 0.01 seconds
    Outputs:
        RefTraj - 4x4xN matrix of reference trajectory configurations
        gripper_state - vector of 0 or 1 representing the state of the 
            gripper at a given time
%}

    % length of each time segment
    t_segment = [3 2 2 2 3 2 2 2]*k/0.01;
    
    % initialize trajectory
    RefTraj = zeros(4,4,sum(t_segment));
    gripperstate = zeros(1,sum(t_segment));
    
    
    
    % initial to standoff
    T_se1 = CartesianTrajectory(Tse_initial, ...
        Tsc_initial*Tce_standoff, t_segment(1)*0.01/k,t_segment(1),5); 
    RefTraj(:,:,1:t_segment(1)) = reshape(cell2mat(T_se1),[4,4,t_segment(1)]);
    T_total = t_segment(1);
    
    % standoff to grasping 
    T_se2 = ScrewTrajectory(Tsc_initial*Tce_standoff, ...
        Tsc_initial*Tce_grasp, t_segment(2)*0.01/k,t_segment(2),5); 
    RefTraj(:,:,T_total+1:T_total+t_segment(2)) = reshape(cell2mat(T_se2), ...
        [4,4,t_segment(2)]);
    T_total = T_total + t_segment(2);
    
    % closing gripper
    T_se3 = CartesianTrajectory(Tsc_initial*Tce_grasp, ...
        Tsc_initial*Tce_grasp, t_segment(3)*0.01/k,t_segment(3),5); 
    RefTraj(:,:,T_total+1:T_total+t_segment(3)) = reshape(cell2mat(T_se3), ...
        [4,4,t_segment(3)]);
    gripperstate(1,T_total+1:T_total+t_segment(3)) = 1;
    T_total = T_total + t_segment(3);
    
    % grasping to standoff
    T_se4 = ScrewTrajectory(Tsc_initial*Tce_grasp, ...
        Tsc_initial*Tce_standoff, t_segment(4)*0.01/k,t_segment(4),5); 
    RefTraj(:,:,T_total+1:T_total+t_segment(4)) = reshape(cell2mat(T_se4), ...
        [4,4,t_segment(4)]);
    gripperstate(1,T_total+1:T_total+t_segment(4)) = 1;
    T_total = T_total + t_segment(4);
    
    % standoff to standoff above final
    T_se5 = CartesianTrajectory(Tsc_initial*Tce_standoff, ...
        Tsc_final*Tce_standoff, t_segment(5)*0.01/k,t_segment(5),5); 
    RefTraj(:,:,T_total+1:T_total+t_segment(5)) = reshape(cell2mat(T_se5), ...
        [4,4,t_segment(5)]);
    gripperstate(1,T_total+1:T_total+t_segment(5)) = 1;
    T_total = T_total + t_segment(5);
    
    % standoff to final object configuration
    T_se6 = ScrewTrajectory(Tsc_final*Tce_standoff, ...
        Tsc_final*Tce_grasp, t_segment(6)*0.01/k,t_segment(6),5); 
    RefTraj(:,:,T_total+1:T_total+t_segment(6)) = reshape(cell2mat(T_se6), ...
        [4,4,t_segment(6)]);
    gripperstate(1,T_total+1:T_total+t_segment(6)) = 1;
    T_total = T_total + t_segment(6);
    
    % opening gripper
    T_se7 = CartesianTrajectory(Tsc_final*Tce_grasp, ...
        Tsc_final*Tce_grasp, t_segment(7)*0.01/k,t_segment(7),5); 
    RefTraj(:,:,T_total+1:T_total+t_segment(7)) = reshape(cell2mat(T_se7), ...
        [4,4,t_segment(7)]);
    gripperstate(1,T_total+1:T_total+t_segment(7)) = 0; 
    T_total = T_total + t_segment(7);
    
    % releasing to standoff
    T_se8 = ScrewTrajectory(Tsc_final*Tce_grasp, ...
        Tsc_final*Tce_standoff, t_segment(8)*0.01/k,t_segment(8),5); 
    RefTraj(:,:,T_total+1:T_total+t_segment(8)) = reshape(cell2mat(T_se8), ...
        [4,4,t_segment(8)]);
    gripperstate(1,T_total+1:T_total+t_segment(8)) = 0;

    
end

