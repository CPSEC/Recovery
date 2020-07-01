function [X0_lo X0_up] = estimate_X0(Ad, Bd, xa, Na, epsilon, past_control)
% xa - last reliable state , na - detection steps, epsilon - 1e-7, array 

table_of_Ad_k = [Ad];
Ad_k = Ad;

for i=2:Na
    Ad_k = Ad_k * Ad;
    table_of_Ad_k = [table_of_Ad_k, Ad_k];
end

tmp = Bd*past_control(Na);
for i=1:Na-1
    tmp = tmp + table_of_Ad_k(i)*Bd*past_control(Na - i);
end

l = zeros(size(Ad,1), 1);
l(1,1) = 1;

e = 1;
for i=1:Na-1
    e = e + sqrt(l'*table_of_Ad_k(i)*table_of_Ad_k(i)'*l);
end

e = e * epsilon;

X0_lo = table_of_Ad_k(Na)*xa + tmp;
X0_up = X0_lo;

X0_lo = X0_lo - e;
X0_up = X0_up + e;

end