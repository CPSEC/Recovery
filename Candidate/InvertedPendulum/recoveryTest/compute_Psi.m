function Psi = compute_Psi(A, delta, order)

Psi = eye(size(A,1)) * delta;

M = A*delta;
tmp = M;
coeff = 1;

for i=2:order
    coeff = coeff / i;
    Psi = Psi + coeff * tmp * delta;
    tmp = tmp * M;
end

end