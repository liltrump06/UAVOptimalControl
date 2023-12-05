clc;clear;close all;
Ts=0.01;T=1; % should be checked by feasibility checker;
primitive_num = 50;

constraint_profile.fmax = 25; % m/s^2
constraint_profile.fmin = 5;  % m/s^2
constraint_profile.wmax = 20; % rad/s
constraint_profile.pos_boundarys = [0,4,0,4,0,4]';
ball_profiles = ball_init(Ts);
%% show trajectory of the ball
figure(1)
scatter3(ball_profiles.x(:,4),ball_profiles.x(:,5),ball_profiles.x(:,6),'filled','o','LineWidth',2)
%% initial states

p0 = [0,0,0]';
v0 = [0,0,0]';
a0 = [0,0,0]';
initstate = [p0,v0,a0];

%% final states
for i = 1:primitive_num
    tpoints = T/Ts+1;
    pf = ball_profiles.x(tpoints,4:6)';
    vf = -(ball_profiles.x(tpoints,1:3)/norm(ball_profiles.x(tpoints,1:3),2)*1)';
    af = [0,0,0]';
    da = af-a0;     % difference between states
    dv = vf-v0-a0*T;
    dp = pf-p0-v0*T-0.5*a0*T^2;
    [alpha,beta,gamma] = coeff_derive(T,dp,dv,da);
    [time,jtraj,atraj,vtraj,ptraj] = generate_traj(T,Ts,alpha,beta,gamma,initstate);
    % feasibility_checker()
    [Rot,thrust,angular_vel] = true_dynamics(atraj,jtraj);% calculate real dynamics of the drone 
end
% flag = check_feasibility_lite(constrain_profile,thrust,angular_vel,jtraj);

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
    newPosition = [200,200,600,600];
    set(gcf,'Position',newPosition);
    droneALL = drone_outlook();
    
    for i = 1:size(atraj,2)
        droneNew = Rot(:,:,i)*droneALL+ptraj(:,i);
        scatter3(ball_profiles.x(i,4),ball_profiles.x(i,5),ball_profiles.x(i,6),40,'filled')
        hold on
        scatter3(droneNew(1,:),droneNew(2,:),droneNew(3,:))
        % axis([-2,2,-2,2,-1,4])
        axis([ptraj(1,i)-0.1,ptraj(1,i)+0.1,ptraj(2,i)-0.1,ptraj(2,i)+0.1,ptraj(3,i)-0.1,ptraj(3,i)+0.1])
        view(90,0)
        hold off
        frame = getframe(gcf);
        writeVideo(outputVideo,frame);
    end
    close(outputVideo);
end