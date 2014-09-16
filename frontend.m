clear all
close all

inputs=[.4 0 0 0 .3 .1]';
hexa=UAV()
rates=hexacopterModel(hexa,inputs)
timestep=1;

for i = 1:5
hexa = simulateUAV(hexa, inputs, timestep);
showUAV(hexa)
hold on
pause(1);
end

limits=5;
%axis([-limits, limits, -limits, limits, -limits, limits])