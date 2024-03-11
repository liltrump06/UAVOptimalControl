clc;clear;close all
%%
tic
setParameter;
time_duration = 4;
time_step = quadrotorParameter.Ts;
final_position = [-1,-1,-3]';
[time,jtraj,atraj,vtraj,ptraj] = trajectoryGene(time_duration,time_step,final_position);
% [time,atraj,vtraj,ptraj]=trajgenefixed();
%%
e3 = [0,0,1]';
p0 = [0,0,0]';
v0 = [0,0,0]';
% p0 = [0.6,0,-0.4]';
% v0 = [0,0.12*pi,0]';
R0 = reshape(eye(3),[9,1]);
w0 = [0,0,0]';
% T0 = [0,0,0]';
% tau0 = [0,0,0]';
% r0 = [100,100,1,0]';

r0 = [1140,1140,1140,1140]';
% r0 = [1000,1000,1000,1000]';
[T0,tau0]=rpm2force(r0);
x0 = [p0;v0;R0;w0;T0;tau0;r0];
cur_T = 0;
mpcinput = struct;
preview_length = 5;
xlist = [];
ulist = [];
Flist = [];
Wlist = [];
wdlist = [];
ptraj = [ptraj,ptraj(:,end).*ones(3,preview_length)];
vtraj = [vtraj,vtraj(:,end).*ones(3,preview_length)];
atraj = [atraj,atraj(:,end).*ones(3,preview_length)];
% for i = 1:(size(time,2)-preview_length)
for i = 1:size(time,2)
    if i==1088
        disp(1088);
    end
    mpcinput.N = preview_length;
    mpcinput.pd = ptraj(:,i:i+preview_length);
    mpcinput.vd = vtraj(:,i:i+preview_length);
    mpcinput.ad = atraj(:,i:i+preview_length);
    mpcinput.x = x0;
    mpcoutput = solveMPC(mpcinput);
    [T_hat,wd,wd_dot,W] = getRealInput(mpcoutput,x0);
    x0 = getHighGainInput(T_hat,wd,wd_dot,x0);
    % x0 = getPIDInput(x0,ptraj(:,i),vtraj(:,i));
    % simulate forward step
    [t,x] = ode45(@vdp_quad,[0,time_step],x0);
    x0 = x(end,:)';
    cur_T = cur_T + time_step;
    xlist = [xlist,x0];
    ulist = [ulist,mpcoutput.uopt];
    Flist = [Flist,mpcoutput.F];
    Wlist = [Wlist,W];
    wdlist = [wdlist,wd];
end
toc
%%
% plot(1:200,xlist(3,:),1:200,ptraj(3,1:200))
figure
mean_rpm = sum(xlist(23:26,:),1)/4;
plotlen = 1:size(time,2);
% plot(1:201,mean_rpm)
% u_MPC = load("direct_mpc.mat","ulist").ulist;
% plot(1:200,ulist(3,:))
subplot(3,1,1)
plot(plotlen,xlist(2,:),plotlen,ptraj(2,plotlen),plotlen,xlist(5,:),plotlen,vtraj(2,plotlen))
legend("pos","pos_d")
subplot(3,1,2)
plot(plotlen,ulist(3,:),plotlen,Flist(3,:))
legend("u","F")
subplot(3,1,3)
plot(plotlen,Wlist,plotlen,xlist(19,:))
legend("W","T")
plot(plotlen,Wlist,plotlen,ulist(3,:))
legend("W","u")
%% 
figure
plot(plotlen,xlist(16,:),plotlen,wdlist(1,:))
legend("w_x","wd_x")

figure
scatter3(-xlist(1,:),-xlist(2,:),-xlist(3,:))
hold on 
scatter3(-ptraj(1,:),-ptraj(2,:),-ptraj(3,:))
hold off
legend("pos","pos_d")
axis ([-10,10,-10,10,-10,10])
xlabel("X")
ylabel("Y")
zlabel("Z")
%% 

%%
% xsyms = load("symsreal.mat","xlist").xlist
% plot(1:201,xlist(19,:),1:201,xsyms(19,:))





