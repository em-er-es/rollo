%% EKF ANALYSIS ON ROLLO
% Generates plots from 2 log files
% Log files contain published data from EKF and preprocessor nodes
% First plot shows position of the robot computed acording to odometry, EKF and
% motion capture
% Second plot displays position and orientation of the robot with resect to
% time

close all
clear all

% logtitle = '08fL12R1, EKF q=8,r=32';
% ekflog = 'ros.08fL12R19.280216--090420.EKF.Q=8,R=32.log';
% measlog = 'ros.08fL12R19.280216--090420.Pose2DStamped.Q=8,R=32.log';

% logtitle = '07fL19R12, EKF q=100,r=10'; 
% ekflog = 'ros.07fL19R12.280216--085432.EKF.Q=100,R=10.log';
% measlog = 'ros.07fL19R12.280216--085432.Pose2DStamped.Q=100,R=10.log';

% logtitle = '07fL12R19, EKF q=100,r=10'; 
% ekflog = 'ros.07fL12R19.280216--085041.EKF.Q=100,R=10.log';
% measlog = 'ros.07fL12R19.280216--085041.Pose2DStamped.Q=100,R=10.log';
%  
% logtitle = '06fL12R19, EKF q=10,r=10'; 
% ekflog = 'ros.06fL12R19.280216--084608.EKF.Q=10,R=10.log';
% measlog = 'ros.06fL12R19.280216--084608.Pose2DStamped.Q=10,R=10.log';

% logtitle = '05fL19R12, EKF q=1.0,r=10'; 
% ekflog = 'ros.05fL19R12.280216--084108.EKF.Q=1.0,R=10.log';
% measlog = 'ros.05fL19R12.280216--084108.Pose2DStamped.Q=1.0,R=10.log';


% logtitle = '05fL12R19, EKF q=1.0,r=10'; 
% ekflog = 'ros.05fL12R19.280216--082726.EKF.Q=1.0,R=10.log';
% measlog = 'ros.05fL12R19.280216--082726.Pose2DStamped.Q=1.0,R=10.log';

% logtitle = '04fL12R19, EKF q=0.1,r=10'; 
% ekflog = 'ros.04fL12R19.280216--082131.EKF.Q=0.1,R=10.log';
% measlog = 'ros.04fL12R19.280216--082131.Pose2DStamped.Q=0.1,R=10.log';

% logtitle = '03fL12R19, EKF q=0.1,r=100'; 
% ekflog = 'ros.03fL12R19.280216--081442.EKF.Q=0.1,R=100.0.log';
% measlog = 'ros.03fL12R19.280216--081442.Pose2DStamped.Q=0.1,R=100.0.log';

% logtitle = '02F12, EKF q=0.1,r=1.0'; 
% ekflog = 'ros.02F12.280216--080453.EKF.Q=0.1,R=1.0.log';
% measlog = 'ros.02F12.280216--080453.Pose2DStamped.Q=0.1,R=1.0.log';


logtitle = '01F19, EKF q=0.1,r=0.1'; 
ekflog = 'ros.01F19.280216--075456.EKF.Q=0.1,R=0.1.log';
measlog = 'ros.01F19.280216--075456.Pose2DStamped.Q=0.1,R=0.1.log';

[ekfsec,ekfnsec,ekfx,ekfy,ekfth,odomx,odomy,odomth] = extractEKFpublish(ekflog);
[meassec,measnsec,measx,measy,measth] = extractMEASpublish(measlog);

fig1 = figure;
set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.
grid on
plot(ekfx, ekfy,'rx');
hold on

plot(odomx, odomy, 'g');
hold on

plot(measx, measy, 'b');
legend('state estimate', 'odometry', 'measurement')
title(logtitle);
xlabel('position x [m]')
ylabel('position y [m]')

saveas(fig1, strcat(logtitle,'.jpg'));
saveas(fig1, strcat(logtitle,'.fig'));
saveas(fig1, strcat(logtitle,'.pdf'));
saveas(fig1, strcat(logtitle,'.svg'));



t_ekf_init = ekfsec(1);
t_ekf = ekfsec - ekfsec(1);
t_ekf = t_ekf + ekfnsec/1000000000;

t_meas_init = meassec(1);
t_meas = meassec - meassec(1);
t_meas = t_meas + measnsec/1000000000;

%translate orientation in range -pi to pi
for i = 1:length(odomth)
    if (odomth(i) > pi)
        odomth(i) = odomth(i) - 2*pi;
    else if (odomth(i) < -pi)
             odomth(i) = odomth(i) + 2*pi;
        end
    end
    
end

fig2 = figure;
grid on
title(logtitle);
set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.
subplot(3,1,1)
plot(t_ekf, ekfx, 'rx', t_ekf, odomx, 'g', t_meas, measx, 'b')
title(logtitle);
legend('state estimate', 'odometry', 'measurement')
xlabel('time [s]')
ylabel(['position x [m]'])

subplot(3,1,2)
plot(t_ekf, ekfy, 'rx', t_ekf, odomy, 'g', t_meas, measy, 'b')
legend('state estimate', 'odometry', 'measurement')
xlabel('time [s]')
ylabel(['position y [m]'])

subplot(3,1,3)
plot(t_ekf, ekfth, 'rx', t_ekf, odomth, 'g', t_meas, measth, 'b')
legend('state estimate', 'odometry', 'measurement')
xlabel('time [s]')
ylabel(['orientation [rad]'])

saveas(fig2, strcat(logtitle,' wrt time','.jpg'));
saveas(fig2, strcat(logtitle,' wrt time','.fig'));
saveas(fig2, strcat(logtitle,' wrt time','.pdf'));
saveas(fig2, strcat(logtitle,' wrt time','.svg'));