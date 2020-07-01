global u;

inputs = [ -13.8314
   -9.0344
   20.0000
   20.0000
   12.6819
  -20.0000
  -20.0000];

x = [0.5, 1, pi/4, -1];
result = x;

for i=1:7
   u = inputs(i);
   
   [T y] = ode45(@dynamics, [0 0.1], x);
   
   result = [result; y];
   
   x = y(end,:);
end

plot(result(:,3), result(:,4), 'linewidth', 1.5, 'color', 'b');
hold on;

function dxdt = dynamics(t, x)

global u;

dxdt(1,1) = x(2,1);
dxdt(2,1) = -0.1818*x(2,1) + 2.6727*x(3,1) + 1.8182*u;
dxdt(3,1) = x(4,1);
dxdt(4,1) = -0.4545*x(2,1) + 31.1818*x(3,1) + 4.5455*u;

end