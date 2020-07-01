function [A_disc, B_disc] = LTI_translation(A_cont, B_cont, delta)

A_disc = expm(A_cont*delta);

B_disc = compute_Psi(A_cont, delta, 1000) * B_cont;

end

