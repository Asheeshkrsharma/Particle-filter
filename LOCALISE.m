%Author: Asheesh Sharma
%MSc Robotics, University of Bristol
%License: UIA (Use It Anywhere)
%Disclaimer: The UIA licesne applies to any part of this code except for
%any where it is specifically mentioned. You are given this software for
%free so dont try to sell it and do not bother me if some thing is broken.
%Finally, to all the students out there. The challenge is not to complete a
%task. It is to not plagiarize. :D 
clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
axis equal; %keeps the x and y scale the same
map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map
map=[-30,0;-30,40;30,40;30,60;5,60;45,90;85,60;60,60;60,40;120,40;120,60;95,60;135,90;175,60;150,60;150,40;210,40;210,60;185,60;225,90;265,60;240,60;240,40;300,40;300,0];
%map=[0,0; 20,0; 20,60; 40,60; 40,-25; 100,0; 150,-40; 150,20; 100,20; 100,120; 80,90; 80,30; 60,30; 60,90; 30,90; 30,150; -30,150; -60,120; 0,120; 0,90; 0,90];
% botSim = BotSim(map,[0.01,0.005,0]);  %sets up a botSim object a map, and debug mode on.
%map=[0,0;60,0;60,50;100,50;70,0;110,0;150,80;30,80;30,40;0,80];
botSim = BotSim(map,[0,0,0]);  %sets up a botSim object a map, and debug mode on.
botSim.drawMap();
drawnow;
botSim.randomPose(10); %puts the robot in a random position at least 10cm away from a wall
target = botSim.getRndPtInMap(10);  %gets random target.
%target=[-16.1551 28.5469];
%botSim.setBotPos([225, 75]);
tic %starts timer

%your localisation function is called here.
returnedBot = localise(botSim,map,target); %Where the magic happens

resultsTime = toc %stops timer

%calculated how far away your robot is from the target.
resultsDis =  distance(target, returnedBot.getBotPos())
