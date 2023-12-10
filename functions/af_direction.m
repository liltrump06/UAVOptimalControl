function af_norm = af_direction(vf_norm,theta1)
    theta2 = 0;
    fan = [0,1,0]';
    axis1 = cross(fan,vf_norm);
    q_theta = [cos(theta1/2),sin(theta1/2)*axis1'];
    mid = quatrotate(q_theta,vf_norm');
    q = [cos(theta2/2),sin(theta2/2)*vf_norm'];
    af_norm = quatrotate(q,mid)';
end