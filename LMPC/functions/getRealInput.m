function [Tout,wd,wd_dot] = getRealInput(mpcoutput,x0)
global quadrotorParameter
u = mpcoutput.uopt;
F = mpcoutput.F;
m = quadrotorParameter.m;
c = quadrotorParameter.c;
e3 = quadrotorParameter.e3;
g = quadrotorParameter.g;
b = quadrotorParameter.b;
step = quadrotorParameter.Ts;
v = x0(4:6);
R = reshape(x0(7:15),[3,3]);
T = x0(19);
Vz = e3'*R'*v;
T_hat0 = T+T*c*Vz;
w=x0(16:18);
wz = w(3);
% omega = sym('omega',[3,1]);
% syms W real
% omega(3) = wz;
% Omega = [0 -omega(3) omega(2);omega(3) 0 -omega(1);-omega(2) omega(1) 0];


gamma = (1+c*e3'*R'*v)*R*e3 + c*v;
eta = -c*v'*R*e3+(1+c.*e3'*R'*v);
lambda = -(T*c-m*g*c)/m .*(F-m.*g*c.*v)-T*c.*(e3'*R'*(F-m*g*c.*v)/m).*(R*e3);
% eqn = -W.*gamma - T*eta.*(R*Omega*e3) == -b*F+u-lambda;
% sol = solve(eqn,[W,omega(1),omega(2)]);
% Wopt = double(sol.W);
% wx = double(sol.omega1);
% wy = double(sol.omega2);
pp = (-b*F+u-lambda)/(-T*eta);
qq = -gamma/(T*eta);

aa1 = R(2,1)*R(1,2)/R(1,1)-R(2,2);
bb1 = qq(2)+R(2,1)*qq(1)/R(1,1);
cc1 = pp(2)-R(2,1)*pp(1)/R(1,1);
aa2 = R(3,1)*R(1,2)/R(1,1)-R(3,2);
bb2 = qq(3)+R(3,1)*qq(1)/R(1,1);
cc2 = pp(3)-R(3,1)*pp(1)/R(1,1);
W = (aa1*cc2-cc1*aa2)/(bb1*aa2-aa1*bb2);
wx = (bb1*cc2-cc1*bb2)/(bb1*aa2-aa1*bb2);
wy = (pp(1)+qq(1)*W+R(1,2)*wx)/R(1,1);
wd = [wx,wy,wz]';
wd_dot = (wd-wz)/step;
Tout = T+W*step;
end