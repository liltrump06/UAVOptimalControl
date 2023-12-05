num = 5;
theta1list = linspace(-1,1,num);
theta2list = linspace(-pi,pi,num);
vf = [0,sqrt(2)/2,sqrt(2)/2]';
aflist = [];
for theta1 = theta1list
    for theta2 = theta2list
        af_norm = af_direction(vf,theta1,theta2);
        aflist = [aflist,af_norm];
        if(theta1 == 0)&&(theta2 == 0)
            af_special = af_norm;
        end
    end
end
figure(1)
quiver3(zeros(1,num^2),zeros(1,num^2),zeros(1,num^2),aflist(1,:),aflist(2,:),aflist(3,:))
hold on 
quiver3(0,0,0,af_special(1),af_special(2),af_special(3))
hold off
