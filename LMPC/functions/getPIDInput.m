function x = getPIDInput(x0,pd,vd)
global quadrotorParameter
g = quadrotorParameter.g;

Kp = 200;
Kd = 2.5;
p = x0(1:3);
v = x0(4:6);

T = Kp*(p(3)-pd(3))+ Kd* (v(3)-vd(3))+g(3);
tau = [0,0,0]';
r = force2rpm(T,tau);
x = x0;
x(23:26) = r;
x(19) = T;
x(20:22) = tau;

