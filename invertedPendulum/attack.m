function x1 = attack(type,x)
    if strcmp(type,'add')
        x1 = x - [0.001; 0; 0.001; 0];
    elseif strcmp(type,'multiply')
        x1 = x * 1.001;
    elseif strcmp(type,'force')
        x1 = [0.999; 0; pi-0.001; 0];
    end
    
end