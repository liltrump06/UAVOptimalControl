% Simple minimum_jerk trajectory design using PMP
function [time,jtraj,atraj,vtraj,ptraj] = trajectoryGene(T,Ts,pf)
p0 = [0,0,0]';
v0 = [0,0,0]';
a0 = [0,0,0]';
af = a0;
vf = v0;
initstate = [p0,v0,a0];
[dp,dv,da] = state_diff(T,p0,pf,v0,vf,a0,af);
[alpha,beta,gamma] = coeff_derive(T,dp,dv,da);
[time,jtraj,atraj,vtraj,ptraj] = generate_traj(T,Ts,alpha,beta,gamma,initstate);

