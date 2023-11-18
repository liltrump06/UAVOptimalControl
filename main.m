clc;clear;close all;
Ts=0.01;T=1; % should be checked by feasibility checker;
tpoints = T/Ts+1;
constrain_profile.fmax = 25; % m/s^2
constrain_profile.fmin = 5;  % m/s^2
constrain_profile.wmax = 20; % rad/s
constrain_profile.pos_boundarys = [0,4,0,4,0,4]';
ball_profiles = ball_init(T,Ts);
%%
figure(1)
scatter3(ball_profiles.x(:,4),ball_profiles.x(:,5),ball_profiles.x(:,6),'filled','o','LineWidth',2)
%% initial states
p0 = [0,0,0]';
v0 = [0,0,0]';
a0 = [0,0,0]';
initstate = [p0,v0,a0];
%% final states
% pf = [1,1,0]';
% vf = [0,0,0]';
pf = ball_profiles.x(tpoints,4:6)';
vf = -(ball_profiles.x(tpoints,1:3)/norm(ball_profiles.x(tpoints,1:3),2)*1)';
af = [0,0,0]';

%% difference between states
da = af-a0;
dv = vf-v0-a0*T;
dp = pf-p0-v0*T-0.5*a0*T^2;
%% solving for the coefficients 
[alpha,beta,gamma] = coeff_derive(T,dp,dv,da);

%% generate trajectory 
[time,jtraj,atraj,vtraj,ptraj] = generate_traj(T,Ts,alpha,beta,gamma,initstate);
[Rot,thrust,angular_vel] = true_dynamics(atraj,jtraj);
%% feasibility checker
flag = check_feasibility(constrain_profile,thrust,angular_vel,jtraj);

%% trajectory record
if flag == false
    disp("\n infeasible primitives \n ")
else
    videoFile = 'outputVideo1000hz.mp4';  % Choose a filename for your video
    outputVideo = VideoWriter(videoFile,'MPEG-4');

    outputVideo.FrameRate = 100;  % Adjust the frame rate as needed
    open(outputVideo);
    figure(2)
    % hold on 
    grid on
    set(gcf,'outerposition',get(0,'screensize'));
    dronex = linspace(-0.05,0.05,20);  % adjust the number of points as needed
    droney = linspace(-0.05,0.05,20);    % adjust the number of points as needed
    [droneX, droneY] = meshgrid(dronex, droney);
    droneZ = zeros(20,20);
    droneX=reshape(droneX,1,20*20);
    droneY=reshape(droneY,1,20*20);
    droneZ=reshape(droneZ,1,20*20);
    droneALL = [droneX;droneY;droneZ];
    for i = 1:size(atraj,2)
        droneNew = Rot(:,:,i)*droneALL+ptraj(:,i);
        scatter3(ball_profiles.x(i,4),ball_profiles.x(i,5),ball_profiles.x(i,6),'filled','o','LineWidth',2)
        hold on
        scatter3(droneNew(1,:),droneNew(2,:),droneNew(3,:))
        axis([-2,2,-2,2,-1,4])
        view(90,0)
        hold off
        frame = getframe(gcf);
        writeVideo(outputVideo,frame);
    end
close(outputVideo);
end