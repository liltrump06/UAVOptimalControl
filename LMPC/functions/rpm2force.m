function [T,tau] = rpm2force(r)
global quadrotorParameter

C0 = quadrotorParameter.C0;
C1 = quadrotorParameter.C1;
l = quadrotorParameter.l;
k = quadrotorParameter.k;

Tp = zeros(4,1);
for i = 1:4
    Tp(i) = C0 *r(i)+C1*r(i)^2;
end

coeffMatrix = [[1,1,1,1];[l,0,-l,0];[0,-l,0,l];[k,-k,k,-k]];
p = coeffMatrix*Tp;
T = p(1);
tau = p(2:4);
