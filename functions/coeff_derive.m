function [alpha,beta,gamma] = coeff_derive(T,dp,dv,da)
%COEFF_DERIVE Summary of this function goes here
%   Detailed explanation goes here
for i = 1:3
    diff_vec = [dp(i);dv(i);da(i)];
    coeff_derive_mat = [720,-360*T,60*T^2;-360*T,168*T^2,-24*T^3;60*T^2,-24*T^3,3*T^4]/T^5;
    coeff_vec = coeff_derive_mat*diff_vec;% [alpha,beta,gamma]
    alpha(i) = coeff_vec(1);
    beta(i) = coeff_vec(2);
    gamma(i) = coeff_vec(3);
end

end

