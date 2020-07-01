function result = recovery_control(A_dyn, B_dyn, k, initial_set_lo, initial_set_up, target_set_lo, target_set_up, safe_set_lo, safe_set_up, control_up, control_lo)

%A_dyn
%B_dyn

n = size(A_dyn,1);

numOfVars = n*(k+1) + k;
numOfCons = 2*numOfVars;

A = zeros(numOfCons, numOfVars);
b = zeros(numOfCons, 1);

% initial set

for i=1:n
   A(i, i) = -1;
   A(i+n, i) = 1;
   b(i, 1) = -initial_set_lo(i,1);
   b(i+n, 1) = initial_set_up(i,1);
end

startPos = 2*n;
lastPos = n*k;

% target set
for i=1:n
   A(startPos+i, lastPos+i) = -1;
   A(startPos+i+n, lastPos+i) = 1;
   b(startPos+i, 1) = -target_set_lo(i,1);
   b(startPos+i+n, 1) = target_set_up(i,1);
end

% safe set
for i=2:k
    for j=1:n
        A(2*i*n+j, (i-1)*n+j) = -1;
        A(2*i*n+n+j, (i-1)*n+j) = 1;
        b(2*i*n+j, 1) = -safe_set_lo(j,1);
        b(2*i*n+n+j, 1) = safe_set_up(j,1);
    end
end
    



% ranges for the control inputs
controlPos = n*(k+1);
startPos = 2*n*(k+1);
for i=1:k
   A(i+startPos, controlPos+i) = -1;
   A(i+startPos+k, controlPos+i) = 1;
   b(i+startPos, 1) = -control_lo;
   b(i+startPos+k, 1) = control_up;
end

% construct the constraints for the dynamics
numDynCons = k*n;
Aeq = zeros(numDynCons, numOfVars);
beq = zeros(numDynCons, 1);


% define the one-step transitions

pos_u = n*(k+1) + 1;
pos_i = 0;
pos_j = 0;

while pos_i < n*k
    for i=1:n
        for j=1:n
            Aeq(pos_i+i, pos_j+j) = -A_dyn(i, j);
        end
   
        Aeq(pos_i+i, pos_j+n+i) = 1;
        Aeq(pos_i+i, pos_u) = -B_dyn(i, 1);
    end

    pos_i = pos_i + n;
    pos_j = pos_j + n;
    pos_u = pos_u + 1;
end



%Aeq
%beq
A
b

f = ones(1, numOfVars);

result = linprog(f, A, b, Aeq, beq)

end