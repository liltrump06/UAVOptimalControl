clc;clear;close all;
Ts=0.01; % should be checked by feasibility checker;
scale = 0.02;
num_T1 = 21;
num_theta1 = 25;
num_thrust = 10;
num_T2 = 5;
constraint_profile.fmax = 25; % m/s^2
constraint_profile.fmin = 5;  % m/s^2
constraint_profile.wmax = 20; % rad/s
constraint_profile.pos_boundarys = [0,4,0,4,0,4]';
Tmin = 0.5;
Tmax = 1.5;
T1list = linspace(Tmin,Tmax,num_T1);
T2list = linspace(0.2,0.6,num_T2);
theta1list = linspace(-pi/2,pi/2,num_theta1);
thrustlist = linspace(constraint_profile.fmin+1,constraint_profile.fmax-1,num_thrust);
p0 = [0,0,0]';
v0 = [0,0,0]';
a0 = [0,0,0]';
af2 = a0;
vf2 = v0;
initstate = [p0,v0,a0];
ball_profiles = ball_init(Ts);
%%
aflist = [];
pflist = [];
split=[];
ptraj_list={};
Rot_list={};
J=inf;
bestcnt = 0;
failcnt = 0;
passcnt = 0;
% ptraj_list_z=[];
cnt = 1;
total_primitives = 0;
tic
for T=T1list
    for theta1=theta1list
        for thrust_f=thrustlist
            total_primitives = total_primitives+1;
            tpoints = round(T/Ts)+1;
            vf = -(ball_profiles.x(tpoints,1:3)/norm(ball_profiles.x(tpoints,1:3),2))';
            % vf is the normalized vec for af;
            af_norm = af_direction(vf,theta1);
            [f_norm,amp,flag]= thrust2acc_amp(af_norm,thrust_f);
            if flag == false
                continue
            end
            af = amp*af_norm;
            pf = ball_profiles.x(tpoints,4:6)'-scale*f_norm;
            % quiver3(pf(1),pf(2),pf(3),af(1),af(2),af(3));
            aflist = [aflist,af];
            pflist = [pflist,pf];
            [dp,dv,da] = state_diff(T,p0,pf,v0,vf,a0,af);

            [alpha,beta,gamma] = coeff_derive(T,dp,dv,da);
            check_result = check_feasibility(alpha,beta,gamma,0,T,a0,constraint_profile);
            if check_result == 0
                continue
            end
            for T2 = T2list
                [~,dv2,da2] = state_diff(T2,0,0,vf,vf2,af,af2);
                [alpha2,beta2,gamma2] = coeff_derive_Pfree(T2,dv2,da2);
                check_result2 = check_feasibility(alpha2,beta2,gamma2,0,T2,af,constraint_profile);
                if check_result2 == 0
                    continue
                end
                passcnt = passcnt+1;
                initstate2 = [pf,vf,af];
                Jnow1 = compute_cost(alpha,beta,gamma,T);
                Jnow2 = compute_cost(alpha,beta,gamma,T2);
                Jnow = Jnow1+Jnow2;
                if J > Jnow
                    bestcnt = bestcnt+1;
                    J = Jnow;
                else
                    continue
                end
                [time,jtraj,atraj,vtraj,ptraj] = generate_traj(T,Ts,alpha,beta,gamma,initstate);
                [time2,jtraj2,atraj2,vtraj2,ptraj2] = generate_traj(T2,Ts,alpha2,beta2,gamma2,initstate2);
                [Rot,thrust,angular_vel] = true_dynamics(atraj,jtraj);
                [Rot2,thrust2,angular_vel2] = true_dynamics(atraj2,jtraj2);
                % concatnate
                N1 = size(time,2)-1;
                N2 = size(time2,2);
                Rotcat = cat(3,Rot(:,:,1:N1),Rot2);
                ptrajcat = cat(2,ptraj(:,1:N1),ptraj2);
                ptraj_list{bestcnt} = ptrajcat;
                Rot_list{bestcnt} = Rotcat;
                split{bestcnt} = N1;
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
        plot3(ptraj_best(1,:),ptraj_best(2,:),ptraj_best(3,:),'r--')
        
        if i <= split{bestcnt}
            scatter3(ball_profiles.x(i,4),ball_profiles.x(i,5),ball_profiles.x(i,6),50,'filled')
        else
            scatter3(ptraj_best(1,i),ptraj_best(2,i),ptraj_best(3,i),50,'filled')
        end
        scatter3(droneNew(1,:),droneNew(2,:),droneNew(3,:),10,'filled')
        axis equal
        axis([-0.5,2,-0.5,2,-0.5,2])
        % axis([ptraj_best(1,i)-0.1,ptraj_best(1,i)+0.1,ptraj_best(2,i)-0.1,ptraj_best(2,i)+0.1,ptraj_best(3,i)-0.1,ptraj_best(3,i)+0.1])
        view(90,0)
        hold off
        frame = getframe(gcf);
        writeVideo(outputVideo,frame);
    end
close(outputVideo);