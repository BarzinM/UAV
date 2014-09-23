clear all
close all

steps=20;

inputs=[0 0 9.8 0 0 .03]';
hexa=UAV();
hexa = setOrientation(hexa, [0, .1, 0]);
timestep=1;

position=zeros(3,steps+1);
orientation=zeros(3,steps+1);
velocity=zeros(3,steps+1);
rotation=zeros(3,steps+1);
position(:,1)=getPosition(hexa);
orientation(:,1)=getOrientation(hexa);
velocity(:,1)=getVelocity(hexa);
rotation(:,1)=getAngularRate(hexa);

for i = 1:steps
    clc
    fprintf('In time step %0.f:\n',i);
    if(i==10)input(6)=0;end
    hexa = simulateExtendedUAV(hexa, inputs, timestep);
    showExtendedUAV(hexa);
    daspect([1,1,1]);
    
    position(:,i+1)=getPosition(hexa);
    orientation(:,i+1)=getOrientation(hexa);
    velocity(:,i+1)=getVelocity(hexa);
    rotation(:,i+1)=getAngularRate(hexa);

    pause(.2);
end

horizontal=0:steps;

figure;
subplot(4,1,1)
plot(horizontal,position);
legend('X','Y','Z');
title('Position');

subplot(4,1,2)
plot(horizontal,orientation);
legend('X','Y','Z');
title('Orientation');

subplot(4,1,3)
plot(horizontal,velocity);
legend('X','Y','Z');
title('Velocity in body coordinate');

subplot(4,1,4)
plot(horizontal,rotation);
legend('X','Y','Z');
title('Rotation velocity in body coordinate');







