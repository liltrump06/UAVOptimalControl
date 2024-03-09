clc;clear;close all
%%
tic
setParameter;
time_duration = 1;
time_step = quadrotorParameter.Ts;
final_position = [0,0,-1]';
[time,jtraj,atraj,vtraj,ptraj] = trajectoryGene(time_duration,time_step,final_position);
%%
e3 = [0,0,1]';
p0 = [0,0,0]';
v0 = [0,0,0]';
R0 = reshape(eye(3),[9,1]);
w0 = [0,0,0]';
% T0 = [0,0,0]';
% tau0 = [0,0,0]';
% r0 = [100,100,1,0]';

r0 = [100,100,100,100]';
[T0,tau0]=rpm2force(r0);
x0 = [p0;v0;R0;w0;T0;tau0;r0];
cur_T = 0;
mpcinput = struct;
preview_length = 5;
xlist = [x0];
ulist = [];
ptraj = [ptraj,ptraj(:,end).*ones(3,100+preview_length)];
vtraj = [vtraj,vtraj(:,end).*ones(3,100+preview_length)];
atraj = [atraj,atraj(:,end).*ones(3,100+preview_length)];
% for i = 1:(size(time,2)-preview_length)
for i = 1:200
    mpcinput.N = preview_length;
    mpcinput.pd = ptraj(:,i:i+preview_length);
    mpcinput.vd = vtraj(:,i:i+preview_length);
    mpcinput.ad = atraj(:,i:i+preview_length);
    mpcinput.x = x0;
    mpcoutput = solveMPC(mpcinput);
    [T_hat,wd,wd_dot] = getRealInput(mpcoutput,x0);
    x0 = getHighGainInput(T_hat,wd,wd_dot,x0);
    % x0 = getPIDInput(x0,ptraj(:,i),vtraj(:,i));
    % simulate forward step
    [t,x] = ode45(@vdp_quad,[0,time_step],x0);
    x0 = x(end,:)';
    cur_T = cur_T + time_step;
    xlist = [xlist,x0];
    ulist = [ulist,mpcoutput.uopt];
end
toc
%%
plot(1:201,xlist(3,:),1:201,ptraj(3,1:201))
figure
mean_rpm = sum(xlist(23:26,:),1)/4;
% plot(1:201,mean_rpm)
% u_MPC = load("direct_mpc.mat","ulist").ulist;
% plot(1:200,ulist(3,:))
plot(1:201,xlist(20:22,:))





