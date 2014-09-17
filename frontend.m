clear all
close all

inputs=[0 0 .3 0 0 0]';
hexa=UAV();
timestep=1;

for i = 1:30
    clc
    fprintf('In time step %0.f:\n',i);
    hexa = simulateUAV(hexa, inputs, timestep);
    showUAV(hexa);
    limits=10;
    axis([-10 10 -10 10 0 20]);
    %axis([-limits, limits, -limits, limits, -limits, limits])
    hold on
    pause(1);
end
