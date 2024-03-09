function Fd = Fd_gene(v,a)
global quadrotorParameter
g = quadrotorParameter.g;
c = quadrotorParameter.c;
m = quadrotorParameter.m;
Fd =  m*a+c*m*g.*v;

