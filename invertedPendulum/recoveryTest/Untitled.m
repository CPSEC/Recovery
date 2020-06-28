close all
clear all
clc

A = [0 1 0 0; 0 -0.1818 2.6727 0; 0 0 0 1; 0 -0.4545 31.1818 0];
B = [0; 1.8182; 0; 4.5455];


dt = 0.1;


[Ad, Bd] = LTI_translation(A,B,dt);

x0 = [0.5; 1; pi/4; -1];    % initial condition
xr = [0; 0; pi/2; 0];      % reference position

k = 7;
initial_set_lo = x0;
initial_set_up = x0;
errorbound = [100; 0.1; 0.1; 0.1]
target_set_lo = xr - errorbound;
target_set_up = xr + errorbound;

utemp = recovery_control(Ad, Bd, k, initial_set_lo, initial_set_up, target_set_lo, target_set_up);
inputs = utemp((length(utemp)-k+1):end)


result = x0;
x = x0;

for i=1:k
   u = inputs(i);
   [T y] = ode45(@dynamics, [0 0.1], x);
   result = [result; y];
   x = y(end,:);
end

plot(t_span,x(:,1),'r',t_span,x(:,3),'k')
legend('loc','angle')

function dxdt = dynamics(t, x)

global A;
global B;
global u;
disp(size(A));
disp(size(B));
disp(size(x));
disp(size(u));
dxdt = A*x + B*u;

end
