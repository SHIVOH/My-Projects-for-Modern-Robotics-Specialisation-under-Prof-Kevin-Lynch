function state_new = NextState(b, ctrl, dt, vlimit)


% allocate space
state_new = zeros(12,1);

% set velocity limit
ctrl(ctrl > vlimit) = vlimit(ctrl > vlimit);
ctrl(ctrl < -vlimit) = vlimit(ctrl < -vlimit);

% calculate new joint angles and wheel angles
state_new(4:12) = b(4:12)+ctrl*dt;

%parameters of Youbot
r = 0.0475;
l = 0.47/2;
w = 0.3/2;

% define H psuedo inverse matrix
H_p = r/4*[-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w);
     1 1 1 1;
    -1 1 -1 1];

% calculate chassis configuration
V_b = H_p*ctrl(6:9)*dt;
% compute transformation to world framme
phi = b(1);
mat_q = [1,0,0;0,cos(phi),-sin(phi);0,sin(phi),cos(phi)];

% compute delta q
if V_b(1) == 0 %if w is zero
    dq_b = [0; V_b(2); V_b(3)];
else
    dq_b = [V_b(1); %if w is notzero
        (V_b(2)*sin(V_b(1))+V_b(3)*(cos(V_b(1))-1))/V_b(1);
        (V_b(3)*sin(V_b(1))+V_b(2)*(1-cos(V_b(1))))/V_b(1)];
end


% calculate chassis config update
state_new(1:3) = b(1:3) + mat_q*dq_b;

end