%% Name: modeRollo.m
% Author: Rabbia Asghar
% Date: 25/2/2016
% Description: analyze logs to estimate control input in absence of encoders
% extracts angular velocity of the robot and robot wheels
% 

% READ FROM LOGFILE
clear all
close all

[t,x,y,theta] = extract_meas('ros.T45VL31VR38.f5-4.240216.060731.log',100,150);
testname = 'T45VL31VR38.f5-4';
%plot original log and then line of best fit
fig1 = figure;
set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.

plot (x,y);
title(testname);
xlabel(' x in mm ');
ylabel(' y in mm ');
grid on 
p = polyfit(x,y,2);
hold on
x_end = length(x);
y_new = polyval(p, x(1:x_end));
plot(x,y_new);

% once we have a straight line of best fit, find delta S
S = sqrt((min(x) - max(x))^2 + (min(y_new) - max(y_new))^2 );
S_in_m = S/1000;
total_time = t(x_end) - t(1)
deltaS = S_in_m/total_time;
 

%determine length in steps instead
sort_points = [x y_new];
sort_points = sortrows(sort_points , 1);
linelength = 0;
for i = 1:length(x)-1
%     x_i = sort_points(i,1)
%     x_i1 = sort_points(i+1,1)    
     dx = sort_points(i+1,1) - sort_points(i,1);
%     y_i = sort_points(i,2)
%    Y_11 = sort_points(i+1,2)
    dy = sort_points(i+1,2) - sort_points(i,2);
    steplength = sqrt( dx^2 + dy^2 );
    linelength = linelength + steplength;
end
linelength
linelength_m = linelength/1000;
delta_linelength = linelength_m / total_time


deltaThetaLogDeg = (theta(end) - theta(1)) %*pi/180;
omega = deltaThetaLogDeg *pi/(180*total_time)

saveas(fig1, strcat(testname,'.jpg'));
saveas(fig1, strcat(testname,'.png'));