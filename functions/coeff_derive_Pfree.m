function [alpha,beta,gamma] = coeff_derive_Pfree(T,dv,da)
%COEFF_DERIVE Summary of this function goes here
%   Detailed explanation goes here
for i = 1:3
    diff_vec = [dv(i);da(i)];
    coeff_derive_mat = [0,0;-12,6*T;6*T,-2*T^2]/T^3;
    coeff_vec = coeff_derive_mat*diff_vec;% [alpha,beta,gamma]
    alpha(i) = coeff_vec(1);
    beta(i) = coeff_vec(2);
    gamma(i) = coeff_vec(3);
end

end