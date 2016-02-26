%analyze logs to estimate control input in absence of encoders
%READ FROM LOGFILE
clear all
%close all
[t,x,y,theta] = extract_meas('ros.T12L.L0.240216.000133.log',100,800);

%plot original log and then line of best fit
figure
plot(t,theta)
title('T12L');
xlabel(' t in s ');
ylabel(' theta in degrees ');
grid on 
p = polyfit(t,theta,1);
 hold on
% 
 x_end = length(x);
total_time = t(x_end) - t(1)


theta_new = polyval(p, t(1:x_end));
plot(t,theta_new);

deltaThetaLogDeg = (theta_new(x_end) - theta_new(1)) %*pi/180;

omega = deltaThetaLogDeg *pi/(180*total_time)

% 
% % once we have line of best fit, find delta S
% S = sqrt((x(x_end) - x(1))^2 + (y_new(x_end) - y_new(1))^2 )
% %S = sqrt((x(end) - x(1))^2 + (polyval(p, x(end)) - polyval(p, x(1)))^2 )
% S_in_m = S/1000;
% total_time = t(x_end) - t(1)
% deltaS = S_in_m/total_time
% %delta S = S/total time
% 
% % % 
% leng_straight=0;
% m=length(x);
% % there are m-1 splines for m points
% for i=1:1:m-1
% dx=x(i+1)-x(i);
% dy= y_new(i+1)-y_new(i);
% leneach=sqrt(dx^2+dy^2);
% leng_straight=leng_straight+leneach;
% end
% 
% 
% 
% % finding length of the curve
% leng_straight = 0;
% for i=1:(x_end-1)
% dx = x(i+1)-x(i);
% dy = y_new(i+1)-y_new(i);
% leneach = sqrt(dx^2 + dy^2);
% leng_straight=leng_straight + leneach;
% end
% 
% leng_straight
% S
% % deltaS = leng_straight/(1000*total_time)
% % x(1)
% % x(x_end)
% % y_new(1)
% % y_new(x_end)

% 
% % finding length of the curve 
% leng_straight = 0;
% dx=x(x_end)-x(1);
% dy= y_new(x_end)-y_new(1);
% leneach=sqrt(dx^2+dy^2);
% leng_straight=leng_straight+leneach;
% 
% leng_straight
% for delta theta?
%deltaTheta = atan2 (y_new(end) - y_new(1), x(end) - x(1) )
% atan(yfinal-y1,xfinal-x1)
