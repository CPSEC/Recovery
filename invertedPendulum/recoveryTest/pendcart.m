function dxdt = pendcart(t,x)

global A;
global B;
global u;
% Sx = sin(x(3));
% Cx = cos(x(3));
% D = m*L*L*(M+m*(1-Cx^2));
% 
% dx(1,1) = x(2);
% dx(2,1) = (1/D)*(-m^2*L^2*g*Cx*Sx + m*L^2*(m*L*x(4)^2*Sx - d*x(2))) + m*L*L*(1/D)*u;
% dx(3,1) = x(4);
% dx(4,1) = (1/D)*((m+M)*m*g*L*Sx - m*L*Cx*(m*L*x(4)^2*Sx - d*x(2))) - m*L*Cx*(1/D)*u;
disp(size(A));
disp(size(B));
disp(size(x));
disp(size(u));

dxdt = A*x + B*u;
% dxdt(1,1) = x(2,1);
% dxdt(2,1) = -0.1818*x(2,1) + 2.6727*x(3,1) + 1.8182*u;
% dxdt(3,1) = x(4,1);
% dxdt(4,1) = -0.4545*x(2,1) + 31.1818*x(3,1) + 4.5455*u;
