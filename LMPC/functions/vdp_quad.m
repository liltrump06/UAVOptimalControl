function dxdt = vdp_quad(t,x)
global quadrotorParameter
p = x(1:3);
v = x(4:6);
R = x(7:15);
R = reshape(R,[3,3]);
w = x(16:18);
T = x(19);
tau = x(20:22);
r = x(23:26);
% 26 state
wx = [0 -w(3) w(2);w(3) 0 -w(1);-w(2) w(1) 0];

%%
c = quadrotorParameter.c;
e3 = quadrotorParameter.e3;
m = quadrotorParameter.m;
g = quadrotorParameter.g;
I = quadrotorParameter.I;
Ir = quadrotorParameter.Ir;
%%
pdot = v;
Vz = e3'*R'*v;
T_hat = T+T*c*Vz;
vdot = g*e3 - (T_hat*R*e3)/m - (c*T*v)/m;
Rdot = R*wx;
Ga = -(-r(1)*Ir*(cross(w,e3))+r(2)*Ir*(cross(w,e3))-r(3)*Ir*(cross(w,e3))+r(4)*Ir*(cross(w,e3)));
wdot = I\(-wx*I*w+Ga+tau);
Rdot = reshape(Rdot,[9,1]);
% Tdot = T;
% rdot = r;
% taudot = tau;
Tdot = 0;
rdot = zeros(4,1);
taudot = zeros(3,1);
dxdt = [pdot;vdot;Rdot;wdot;Tdot;taudot;rdot];
