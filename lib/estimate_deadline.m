function k = estimate_deadline(Ad, Bd, xa, Na, epsilon, past_control, future_control_value, safe_set_lo, safe_set_up, max_k)
% future_control  value  when detected, mak_k - 20

n = size(Ad,1);
table_of_Ad_k = [Ad];
Ad_k = Ad;

k = 0;

for i=2:max_k
    Ad_k = Ad_k * Ad;
    table_of_Ad_k = [table_of_Ad_k, Ad_k];
end

control_inputs = [past_control ones(1,max_k-Na)*future_control_value];

part2 = zeros(n,1);

part3 = zeros(n,1);

for i=1:max_k
    part1 = table_of_Ad_k(i)*xa;
    part2 = Ad*part2 + Bd*control_inputs(i);
    
    tmp = part1 + part2;
    
    for j=1:n
        l = zeros(n, 1);
        l(j,1) = 1;
        
        part3(j,1) = part3(j,1) + sqrt(l'*table_of_Ad_k(i)*table_of_Ad_k(i)'*l)*epsilon;
           
        if tmp(j,1) + part3(j,1) > safe_set_up
            return;
        elseif tmp(j,1) - part3(j,1) < safe_set_lo
            return;
        end
    end
    
    k = k+1;
end

end