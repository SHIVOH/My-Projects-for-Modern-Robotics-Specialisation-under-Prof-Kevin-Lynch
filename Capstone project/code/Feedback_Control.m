function [velocity, X_err1] = Feedback_Control(c,RfTraj,RfTraj_next,cumerror,dt)
M = [1,0,0,0.033;0,1,0,0;0,0,1,0.6546;0,0,0,1];
B1=  [0,0,1,0,0.033,0];
B2 = [0,-1,0,-0.5076,0,0];
B3 = [0,-1,0,-0.3526,0,0];
B4 = [0,-1,0,-0.2176,0,0];
B5 = [0,0,1,0,0,0];
Blist =[B1',B2',B3',B4',B5'];
theta = c(1);
x   = c(2);
y   = c(3);
thetalist = [c(4),c(5),c(6),c(7),c(8)]';
Toe = FKinBody(M, Blist, thetalist);
Tbo = [1,0,0,0.1662;0,1,0,0;0,0,1,0.0026;0,0,0,1];
Tsb = [cos(theta),-sin(theta),0,x;sin(theta),cos(theta),0,y;0,0,1,0.0963;0,0,0,1];
X = Tsb*Tbo*Toe; % The current actual end-effector configuration X

%lets create the reference configuration
%error matrix
X_err = MatrixLog6(inv(X)*RfTraj);
w1 = X_err(3,2);
w2 = X_err(1,3);
w3 = X_err(2,1);
x1 = X_err(1,4);
x2 = X_err(2,4);
x3 = X_err(3,4);
X_err1 = [w1,w2,w3,x1,x2,x3]';

%reference twist 
V_r = (1/dt)*MatrixLog6(inv(RfTraj)*RfTraj_next);
w1 = V_r(3,2);
w2 = V_r(1,3);
w3 = V_r(2,1);
x1 = V_r(1,4);
x2 = V_r(2,4);
x3 = V_r(3,4);
Vr = [w1,w2,w3,x1,x2,x3]';
%END EFFECTOR TWIST
Kp = eye(6);
Ki = zeros(6);
V = Adjoint(inv(X)*RfTraj)*Vr + Kp*X_err1 + Ki*(cumerror+X_err1);
%Velocity of joints and wheels
Jbody = JacobianBody(Blist, thetalist);
r = 0.0475;
l = 0.47/2;
w = 0.3/2;
F6 = (r/4)*[-1/(l+w),1/(l+w),1/(l+w),-1/(l+w);1,1,1,1;-1,1,-1,1];
F = [0,0,0,0;0,0,0,0;F6;0,0,0,0];
Jbase = Adjoint(inv(Toe)*inv(Tbo))*F;
Je = [Jbase,Jbody];
velocity = pinv(Je,1.7e-01)*V;

end
