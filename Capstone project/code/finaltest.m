%program tester
clear all; close all; clc;

% inputs:
% initial configuration of the cube in world coordinates
Tsc_initial = [1 0 0 1;0 1 0 0;0 0 1 0.025;0 0 0 1];

% final configuration of the cube in world coordinates (best and overshoot)
Tsc_goal = [0 1 0 0;-1 0 0 -1;0 0 1 0.025;0 0 0 1];

% final configuration of the cube in world coordinates (newtask)    
%Tsc_goal = [0 1 0 1;-1 0 0 -1.5; 0 0 1 0.025; 0 0 0 1];

% initial configuration of the youbot
config_i = [-0.5 -0.5 0.2 -0.1 0.1 -3*pi/4 0.1 0.1 0 0 0 0 0]';

% reference initial configuration of the youbot
config_ref = [0 0 1 0;0 1 0 0;-1 0 0 0.5;0 0 0 1];

%my tests Proportional Gain (Best)
% K_p = 7*[7 0 0 0 0 0;
%     0 5 0 0 0 0;
%     0 0 4 0 0 0;
%     0 0 0 4 0 0;
%     0 0 0 0 1 0;
%     0 0 0 0 0 0.8];
% my tests Integral Gain matrix (best)
% K_i = [6 0 0 0 0 0;
%     0 5 0 0 0 0;
%     0 0 4 0 0 0;
%     0 0 0 3 0 0;
%     0 0 0 0 2 0;
%     0 0 0 0 0 1];
%my tests Proportional Gain (Newtask)
K_p = 6*[8 0 0 0 0 0;
    0 6 0 0 0 0;
    0 0 5 0 0 0;
    0 0 0 4 0 0;
    0 0 0 0 2 0;
    0 0 0 0 0 1];

%my tests Integral Gain matrix (newtask)
K_i = [5 0 0 0 0 0;
    0 4 0 0 0 0;
    0 0 3 0 0 0;
    0 0 0 2 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1];

%my test Proportional Gain(Overshoot)
% K_p = 2*[9 0 0 0 0 0;
%     0 8 0 0 0 0;
%     0 0 7 0 0 0;
%     0 0 0 6 0 0;
%     0 0 0 0 2 0;
%     0 0 0 0 0 1];
% 
% %my test Integral Gain (Overshoot)
% K_i = 6*[6 0 0 0 0 0;
%     0 5 0 0 0 0;
%     0 0 4 0 0 0;
%     0 0 0 3 0 0;
%     0 0 0 0 2 0;
%     0 0 0 0 0 1];

% run simulation
motionplanner(Tsc_initial,Tsc_goal, config_i, config_ref, K_p, K_i);