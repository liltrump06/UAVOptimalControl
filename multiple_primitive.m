clc;clear;close all;
Ts=0.01; % should be checked by feasibility checker;
scale = 0.02;
num_T = 21;
num_theta1 = 25;
num_theta2 = 7;
num_thrust = 10;
constraint_profile.fmax = 25; % m/s^2
constraint_profile.fmin = 5;  % m/s^2
constraint_profile.wmax = 20; % rad/s
constraint_profile.pos_boundarys = [0,4,0,4,0,4]';
Tmin = 0.5;
Tmax = 1.5;
Tlist = linspace(Tmin,Tmax,num_T);
theta1list = linspace(-pi/2,pi/2,num_theta1);
% theta2list = linspace(-pi,pi,num_theta2);
% theta1list=[0];
theta2list=[0];
thrustlist = linspace(constraint_profile.fmin+1,constraint_profile.fmax-1,num_thrust);
p0 = [0,0,0]';
v0 = [0,0,0]';
a0 = [0,0,0]';
initstate = [p0,v0,a0];
ball_profiles = ball_init(Ts);
%%
aflist = [];
pflist = [];
ptraj_list={};
Rot_list={};
J=inf;
bestcnt = 0;
failcnt = 0;
% ptraj_list_z=[];
cnt = 1;
plaincnt = 0;
tic
for T=Tlist
    for theta1=theta1list
        for theta2=theta2list
            for thrust_f=thrustlist
                plaincnt = plaincnt+1;
                tpoints = round(T/Ts)+1;
                vf = -(ball_profiles.x(tpoints,1:3)/norm(ball_profiles.x(tpoints,1:3),2))';
                % vf is the normalized vec for af;
                af_norm = af_direction(vf,theta1,theta2);
                [f_norm,amp,flag]= thrust2acc_amp(af_norm,thrust_f);
                if flag == false
                    continue
                end
                af = amp*af_norm;
                pf = ball_profiles.x(tpoints,4:6)'-scale*f_norm;
                % quiver3(pf(1),pf(2),pf(3),af(1),af(2),af(3));
                aflist = [aflist,af];
                pflist = [pflist,pf];
                da = af-a0;     % difference between states
                dv = vf-v0-a0*T;
                dp = pf-p0-v0*T-0.5*a0*T^2;
                [alpha,beta,gamma] = coeff_derive(T,dp,dv,da);
                check_result = check_feasibility(alpha,beta,gamma,0,T,a0,constraint_profile);
                if check_result == 0
                    failcnt = failcnt+1;
                    continue
                end
                % Jnow = compute_cost(alpha,beta,gamma,T);
                % if J > Jnow
                %     J = Jnow;
                %     bestcnt = cnt;
                % else
                %     continue
                % end

                [time,jtraj,atraj,vtraj,ptraj] = generate_traj(T,Ts,alpha,beta,gamma,initstate);
                [Rot,thrust,angular_vel] = true_dynamics(atraj,jtraj);% calculate real dynamics of the drone 
                flag_lite = check_feasibility_lite(constraint_profile,thrust,angular_vel,jtraj);
                if flag_lite == false
                    disp("wrong")
                end
                Jnow = compute_cost(alpha,beta,gamma,T);
                if J > Jnow
                    J = Jnow;
                    bestcnt = cnt;
                else
                    continue
                end
                ptraj_list{cnt} = ptraj;
                Rot_list{cnt} = Rot;
                cnt = cnt+1;
            end
        end
    end
end
toc
%%
ptraj_best = ptraj_list{bestcnt};
Rot_best = Rot_list{bestcnt};
pfbest = pflist(:,bestcnt);
figure(1)
% quiver3(pflist(1,:),pflist(2,:),pflist(3,:),aflist(1,:),aflist(2,:),aflist(3,:))
% quiver3(pflist(1,1:num_theta1*num_theta2),pflist(1,1:num_theta1*num_theta2),pflist(1,1:num_theta1*num_theta2),aflist(1,1:num_theta1*num_theta2),aflist(2,1:num_theta1*num_theta2),aflist(3,1:num_theta1*num_theta2),100)
% hold on
% mid_vec = round(num_theta1*num_theta2/2);
% quiver3(pflist(1,mid_vec),pflist(2,mid_vec),pflist(3,mid_vec),aflist(1,mid_vec),aflist(2,mid_vec),aflist(3,mid_vec),'r')
axis equal
% 
hold on 
for i = 1: bestcnt
        ptraj = ptraj_list{i};
        plot(ptraj(2,:),ptraj(3,:))
end
hold off
% plot(ptraj_best(2,:),ptraj_best(3,:))
% axis([-2,2,-2,2,-4,4])
%%

videoFile = 'outputVideo1000hz.mp4';  % Choose a filename for your video
    outputVideo = VideoWriter(videoFile,'MPEG-4');
    outputVideo.FrameRate = 30;  % Adjust the frame rate as needed
    open(outputVideo);
    figure(2)
    % hold on 
    grid on
    newPosition = [200,200,800,800];
    set(gcf,'Position',newPosition);
    droneALL = drone_outlook(scale);

    for i = 1:size(ptraj_best,2)
        droneNew = Rot_best(:,:,i)*droneALL+ptraj_best(:,i);
        plot3(ball_profiles.x(:,4),ball_profiles.x(:,5),ball_profiles.x(:,6),'b--')
        hold on
        scatter3(ball_profiles.x(i,4),ball_profiles.x(i,5),ball_profiles.x(i,6),10,'filled')
        scatter3(droneNew(1,:),droneNew(2,:),droneNew(3,:),10,'filled')
        % axis equal
        axis([-3,3,-1,5,-1,8])
        % axis([ptraj_best(1,i)-0.1,ptraj_best(1,i)+0.1,ptraj_best(2,i)-0.1,ptraj_best(2,i)+0.1,ptraj_best(3,i)-0.1,ptraj_best(3,i)+0.1])
        view(90,45)
        hold off
        frame = getframe(gcf);
        writeVideo(outputVideo,frame);
    end
close(outputVideo);
%%
% plot(time,atraj(2,:),time,vtraj(2,:),time,ptraj(2,:))
% legend("atraj","vtraj","ptraj")







