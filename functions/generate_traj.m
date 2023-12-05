function [time,jtraj,atraj,vtraj,ptraj] = generate_traj(T,Ts,alpha_vec,beta_vec,gamma_vec,initstate)
%GENERATE_TRAJ Summary of this function goes here
%   Detailed explanation goes here
atraj=[];
vtraj=[];
ptraj=[];
jtraj=[];
p0 = initstate(:,1);
v0 = initstate(:,2);
a0 = initstate(:,3);

for i = 1:3
    alpha = alpha_vec(i);
    beta = beta_vec(i);
    gamma = gamma_vec(i);
    single_p=[];
    single_v=[];
    single_a=[];
    single_j=[];
    for t = linspace(0,T,round(T/Ts)+1)
        single_p = [single_p,alpha/120*t^5 + beta/24*t^4 + gamma/6*t^3 + a0(i)/2*t^2+v0(i)*t+p0(i)];
        single_v = [single_v,alpha/24*t^4 + beta/6*t^3 + gamma/2*t^2+a0(i)*t+v0(i)];
        single_a = [single_a,alpha/6*t^3 + beta/2*t^2+gamma*t+a0(i)];
        single_j = [single_j,0.5*alpha*t^2 + beta*t+gamma];
    end
    ptraj = [ptraj;single_p];
    vtraj = [vtraj;single_v];
    atraj = [atraj;single_a];
    jtraj = [jtraj;single_j];
end
time = linspace(0,T,round(T/Ts)+1);

