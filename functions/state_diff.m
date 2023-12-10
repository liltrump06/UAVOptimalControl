function [dp,dv,da] = state_diff(T,p0,pf,v0,vf,a0,af)
%STATE_DIFF Summary of this function goes here
%   Detailed explanation goes here
da = af-a0;     % difference between states
dv = vf-v0-a0*T;
dp = pf-p0-v0*T-0.5*a0*T^2;
end

