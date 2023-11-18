function ball_profile = ball_init(T,Ts)
%BALL_INIT Summary of this function goes here
%   Detailed explanation goes here
ball_profile.m = 1;
ball_profile.v0 = [0,1,1]';
D= 0.3;
g=-9.8;
tspan = linspace(0,T,T/Ts+1);
x0 = [ball_profile.v0',0,0,4]';
[t,x] = ode45(@(t,x) vdp1(t,x,D,g),tspan,x0);
ball_profile.t = t;
ball_profile.x = x;
end

