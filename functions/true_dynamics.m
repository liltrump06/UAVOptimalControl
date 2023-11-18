function [Rlist,flist,omegalist] = true_dynamics(atraj,jtraj)
%TRUE_DYNAMICS Summary of this function goes here
%   Detailed explanation goes here

g = [0,0,9.81]';
flist = [];
omegalist = [];
Rlist = zeros([3,3,size(atraj,2)]);
% yawlist = [];
% pitchlist = [];
for i = 1:size(atraj,2)
    f = norm(atraj(:,i)-g);
    yaw = asin(-atraj(2,i)/f);  %alpha
    pitch = asin(-atraj(1,i)/(f*cos(yaw))); %beta
    flist = [flist,f];
    % yawlist =[yawlist,yaw];
    % pitchlist = [pitchlist,pitch]; 
    R = [cos(pitch),sin(yaw)*sin(pitch),cos(yaw)*sin(pitch);
        0, cos(yaw),-sin(yaw);
        -sin(pitch),sin(yaw)*cos(pitch),cos(yaw)*cos(pitch)];
    Rlist(:,:,i) = R;
    omega = 1/f*diag([1,1,0])*R'*jtraj(:,i);
    omegalist = [omegalist,omega];

end

end

