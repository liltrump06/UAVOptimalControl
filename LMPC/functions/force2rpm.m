function r = force2rpm(T,tau)
global quadrotorParameter

C0 = quadrotorParameter.C0;
C1 = quadrotorParameter.C1;
l = quadrotorParameter.l;
k = quadrotorParameter.k;

p = zeros(4,1);
p(1) = T;
p(2:4) = tau;


coeffMatrix = [[1,1,1,1];[l,0,-l,0];[0,-l,0,l];[k,-k,k,-k]];

Tp = coeffMatrix \ p;
r = zeros(4,1);
for i = 1:4
    r(i) = (-C0 + sqrt(C0^2+4*C1*Tp(i)))/2/C1;
end

