function F = F_gene(x)
global quadrotorParameter
g = quadrotorParameter.g;
c = quadrotorParameter.c;
e3 = quadrotorParameter.e3;
m = quadrotorParameter.m;
T = x(19);
v = x(4:6);
R = reshape(x(7:15),[3,3]);
Vz = e3'*R'*v;
T_hat = T+T*c*Vz;


F = m*g.*e3-T_hat*R*e3-(c*T-m*c*g).*v;
