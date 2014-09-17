clear all
close all

steps=20;

inputs=[0 0 9.8 0 0 0]';
hexa=UAV();
timestep=1;

position=zeros(3,steps+1);
orientation=zeros(3,steps+1);
velocity=zeros(3,steps+1);
rotation=zeros(3,steps+1);
position(:,1)=hexa.position;
orientation(:,1)=hexa.orientation;
velocity(:,1)=hexa.states(1:3);
rotation(:,1)=hexa.states(4:end);

for i = 1:steps
    clc
    fprintf('In time step %0.f:\n',i);
    hexa = simulateUAV(hexa, inputs, timestep);
    
    showUAV(hexa);
    daspect([1,1,1])
    
    position(:,i+1)=hexa.position;
    orientation(:,i+1)=hexa.orientation;
    velocity(:,i+1)=hexa.states(1:3);
    rotation(:,i+1)=hexa.states(4:end);

    pause(1);
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







