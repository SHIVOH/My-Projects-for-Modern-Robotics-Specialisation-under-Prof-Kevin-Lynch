%    As all the ki for one object's wrench-balance equations has nothing to
%    do with another object's equations. Therefore the code here treats wrench-balance equations of each object
%    separately, and list all the k solution at the end.If there is any k in this k-list less than one, it's corresponding
%    object will fall, and the whole assembley will collapse. 


close all, clear all, clc
% Assembly collapses
m = [2,25,35;
    5,66,42]
contacts = [1, 0, 0, 0, pi/2, 0.1;
            1, 2, 60, 60, pi, 0.5;
            2, 0, 72, 0, pi/2, 0.5;
            2, 0, 60, 0, pi/2, 0.5]
        
StabilityCheck(m,contacts)      
m = [2,25,35;
    10,66,42]
contacts = [1, 0, 0, 0, pi/2, 0.5;
            1, 2, 60, 60, pi, 0.5;
            2, 0, 72, 0, pi/2, 0.5;
            2, 0, 60, 0, pi/2, 0.5]
        
StabilityCheck(m,contacts)
% new settings 

m = [5,1,5;
    4,10,2]
contacts = [1, 0, 0, 0, pi/2, 0.2;
            1, 2, 5, 5, pi/4, 0.3;
            2, 0, 6, 0, pi/2, 0.4;
            2, 0, 11, 0, pi/2, 0.4]
        
StabilityCheck(m,contacts)

m = [2,1,5;
    4,10,2]
contacts = [1, 0, 0, 0, pi/2, 0.8;
            1, 2, 5, 5, pi/4, 0.6;
            2, 0, 6, 0, pi/2, 0.4;
            2, 0, 11, 0, pi/2, 0.4]
        
StabilityCheck(m,contacts)
