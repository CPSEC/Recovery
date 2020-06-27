function result = recovery_control(A_dyn, B_dyn, k, initial_set_lo, initial_set_up, target_set_lo, target_set_up)

n = size(A_dyn,1);

numOfVars = n*(k+1) + k;
numOfCons = n*4 + k*2;

A = zeros(numOfCons, numOfVars);
b = zeros(numOfCons, 1);

% initial set

for i=1:n
   A(i, i) = -1;
   A(i+n, i) = 1;
   b(i, 1) = -initial_set_lo(i);
   b(i+n, 1) = initial_set_up(i);
end

startPos = 2*n;
lastPos = n*k;

% target set
for i=1:n
   A(startPos+i, lastPos+i) = -1;
   A(startPos+i+n, lastPos+i) = 1;
   b(startPos+i, 1) = -target_set_lo(i);
   b(startPos+i+n, 1) = target_set_up(i);
end

% ranges for the control inputs
controlPos = n*(k+1);
for i=1:k
   A(i+2*n, controlPos+i) = -1;
   A(i+2*n+k, controlPos+i) = 1;
   b(i+2*n, 1) = 20;
   b(i+2*n+k, 1) = 20;
end

% construct the constraints for the dynamics
numDynCons = k*n;
Aeq = zeros(numDynCons, numOfVars);
beq = zeros(numDynCons, 1);


% define the one-step transitions
i = n*(k+1) + 1;
j = 1;

while j <= n*k
    Aeq(j, j) = -A_dyn(1,1);
    Aeq(j, j+1) = -A_dyn(1,2);
    Aeq(j, j+2) = -A_dyn(1,3);
    Aeq(j, j+3) = -A_dyn(1,4);
    
    Aeq(j, j+4) = 1;
    Aeq(j, i) = -B_dyn(1,1);
    beq(j, 1) = 0;
    
    
    Aeq(j+1, j) = -A_dyn(2,1);
    Aeq(j+1, j+1) = -A_dyn(2,2);
    Aeq(j+1, j+2) = -A_dyn(2,3);
    Aeq(j+1, j+3) = -A_dyn(2,4);
    
    Aeq(j+1, j+5) = 1;
    Aeq(j+1, i) = -B_dyn(2,1);
    beq(j+1, 1) = 0;
    
    
    Aeq(j+2, j) = -A_dyn(3,1);
    Aeq(j+2, j+1) = -A_dyn(3,2);
    Aeq(j+2, j+2) = -A_dyn(3,3);
    Aeq(j+2, j+3) = -A_dyn(3,4);
    
    Aeq(j+2, j+6) = 1;
    Aeq(j+2, i) = -B_dyn(3,1);
    beq(j+2, 1) = 0;
    
    
    Aeq(j+3, j) = -A_dyn(4,1);
    Aeq(j+3, j+1) = -A_dyn(4,2);
    Aeq(j+3, j+2) = -A_dyn(4,3);
    Aeq(j+3, j+3) = -A_dyn(4,4);
    
    Aeq(j+3, j+7) = 1;
    Aeq(j+3, i) = -B_dyn(4,1);
    beq(j+3, 1) = 0;
    
    
    i = i+1;
    j = j+n;
end

%Aeq
%beq
%A
%b

f = ones(1, numOfVars);

result = linprog(f, A, b, Aeq, beq)

end