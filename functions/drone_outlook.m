function droneAll = drone_outlook(scale)
%% draw a circle

centerlist = [[-scale,scale];[scale,scale];[-scale,-scale];[scale,-scale]];
theta = linspace(0, 2*pi, 50);
numCircles = 4;
radius = scale;
circleZ = zeros(1,50);
circleAll = [];
% figure(2)
% hold on
for i = 1:numCircles
    centerX = centerlist(i,1);
    centerY = centerlist(i,2);
    circleX = centerX + radius * cos(theta);
    circleY = centerY + radius * sin(theta);
    circleAll = [circleAll,[circleX;circleY;circleZ]];
end
% scatter3(circleAll(1,:),circleAll(2,:),circleAll(3,:),'filled')

%% draw a basket
stickZ = linspace(0,scale,20);
stickX = zeros(1,20);
stickY = zeros(1,20);
stickAll = [stickX;stickY;stickZ];
basketx = linspace(-scale,scale,20);  % adjust the number of points as needed
baskety = linspace(-scale,scale,20);    % adjust the number of points as needed

[basketX, basketY] = meshgrid(basketx, baskety);
basketZ = ones(20,20)*scale;
basketX=reshape(basketX,1,20*20);
basketY=reshape(basketY,1,20*20);
basketZ=reshape(basketZ,1,20*20);
basketAll = [basketX;basketY;basketZ];

half_barX = linspace(-scale,scale,20);
quart_barX_left = ones(1,20)*-scale;
quart_barX_right = ones(1,20)*scale;
half_barY = half_barX;
quart_barY_bottom = ones(1,20)*-scale;
quart_barY_up = ones(1,20)*scale;
barX_single = [half_barX,half_barX,quart_barX_left,quart_barX_right];
barY_single = [quart_barY_bottom,quart_barY_up,half_barY,half_barY];
barZ_single = zeros(1,80)+scale;
barsize = 5;
barAll=[];
interval = 0.0005;

for i = 1:barsize
    barAll = [barAll,[barX_single;barY_single;barZ_single+interval*i]];
end
droneAll = [circleAll,stickAll,basketAll,barAll];
% scatter3(droneAll(1,:),droneAll(2,:),droneAll(3,:))
% axis([-0.1,0.1,-0.1,0.1,-0.1,0.1])
end