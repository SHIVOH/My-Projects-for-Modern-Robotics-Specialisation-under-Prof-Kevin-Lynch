%capstoneproject
[Output1] = trajectoryGenerator();
a =size(Output1,1);
r = 0.0475;
l = 0.47/2;
w = 0.3/2;
conf = [0,l+r,-w,0,0,0,0,0,0,0,0,0,0];
dt =0.01;
output = [];
cumerror =[0,0,0,0,0,0]';
X_err1S = [0,0,0,0,0,0];
for i =1:(a-1)
a = Output1{i};
RfTraj= a{1};
b = Output1{i+1};
RfTraj_next = b{1};
[velocity, X_err1] = Feedback_Control(conf,RfTraj,RfTraj_next,cumerror,dt);
velocity =velocity';
for j =1:size(velocity,2)
    
    if velocity(j)>5
        
        velocity(j)=5;
    elseif velocity(j)<-5
        
        velocity(j)=-5;
    
    end
%     if velocity(5)~=0
%         velocity(5)=0;
%     end
%     if velocity(6)~=0
%         velocity(6)=0;
%     end

end
cumerror = cumerror + X_err1;
X_err1S = [X_err1S; X_err1'];
conf  = NextState(conf,velocity);
velocity'
output = [output;conf];
end
csvwrite('finalprogram1.csv',output)