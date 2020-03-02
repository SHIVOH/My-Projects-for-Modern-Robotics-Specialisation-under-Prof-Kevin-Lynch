function StabilityCheck(m,contacts)

[num_obj,~] = size(m); %number of contacting objects (without ground)[
k = [];
g = 9.81;


for i = 1:num_obj
    % find the contact objects with i th object
    idx_k = [find(contacts(:,1) == i);find(contacts(:,2) == i)];
    
    % the friction cone has two edges, double the contact number
    num_k = length(idx_k)*2;
    
    % linear programming values initialization
    f = ones(num_k,1);
    A = -eye(num_k);
    b = -1*ones(1,num_k);
    F = zeros(3,num_k);
    
    for j = 1:length(idx_k)
        % contact point coordinate
        x = contacts(idx_k(j),3); 
        y = contacts(idx_k(j),4); 
        
        % contact angle,  when the contact object at second column, switch contact angle
        if j<=length(find(contacts(:,1) == i))   
            angle = contacts(idx_k(j),5); 
        else  
            angle = contacts(idx_k(j),5)-pi; 
        end
        
        % contact coefficient
        f_coeff = contacts(idx_k(j),6); 
        
        % the theta between the direct force and the friction cone edges 
        theta = atan2(f_coeff,1);
 
        F(:,2*j-1) = [sin(angle+theta)*x-cos(angle+theta)*y, cos(angle+theta),sin(angle+theta)]';
        F(:,2*j) = [sin(angle-theta)*x-cos(angle-theta)*y, cos(angle-theta),sin(angle-theta)]';
    end
    
    Aeq = F;
    beq = [m(i,1)*m(i,2)*g,0,m(i,1)*g];
    
    %solve all the k which satisify the wrench-balance equations for ith object
    k_temp = linprog(f,A,b,Aeq,beq);
    k = [k;k_temp];
end


eps = 1*10^(-6);  % linear programming error
if length(find(k<1-eps))>=1
    fprintf('k = ')
    k'
    fprintf('the assembly would collapse\n\n\n')
else
    fprintf('k = ')
    k'
    fprintf('the assembly remains standing\n\n\n')
end

end