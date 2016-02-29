function [] = EKFAnalysisFunction(logtitle, ekflog, measlog, filename)
%% takes titles of log files and title for the plots to be genrated as input
% Log files contain published data from EKF and preprocessor nodes
% First plot shows position of the robot computed acording to odometry, EKF and
% motion capture
% Second plot displays position and orientation of the robot with resect to
% time

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
grid on
hL = legend('State estimate', 'Odometry', 'Measurement')
% % Programatically move the Legend
%  newPosition = [0.25 0.768 0.01 0.1];
%  newUnits = 'normalized';
%  set(hL,'Position', newPosition,'Units', newUnits);
title(logtitle,'FontName','Times New Roman','FontSize',16);
xlabel('Position x [m]')
ylabel('Position y [m]')
saveas(fig1, strcat(filename,'.jpg'));
saveas(fig1, strcat(filename,'.fig'));
saveas(fig1, strcat(filename,'.pdf'));
saveas(fig1, strcat(filename,'.svg'));



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
grid on
maxaxy = max([max(ekfx) max(odomx) max(measx)]);
minaxy = min([min(ekfx) min(odomx) min(measx)]);
axis ([0 max(t_ekf)+4 minaxy-0.1 maxaxy+0.1])
title(logtitle,'FontName','Times New Roman','FontSize',16);
hL = legend('State estimate', 'Odometry', 'Measurement')
xlabel('Time [s]')
ylabel(['Position x [m]'])

subplot(3,1,2)
plot(t_ekf, ekfy, 'rx', t_ekf, odomy, 'g', t_meas, measy, 'b')
grid on
hL = legend('State estimate', 'Odometry', 'Measurement')
maxaxy = max([max(ekfy) max(odomy) max(measy)]);
minaxy = min([min(ekfy) min(odomy) min(measy)]);
axis ([0 max(t_ekf)+4 minaxy-0.1 maxaxy+0.1])
% % Programatically move the Legend
% newPosition = [0.945 0.454 0.01 0.1];
% newUnits = 'normalized';
% set(hL,'Position', newPosition,'Units', newUnits);
xlabel('Time [s]')
ylabel(['Position y [m]'])

subplot(3,1,3)
plot(t_ekf, ekfth, 'rx', t_ekf, odomth, 'g', t_meas, measth, 'b')
grid on
hL = legend('State estimate', 'Odometry', 'Measurement')
maxaxy = max([max(ekfth) max(odomth) max(measth)]);
minaxy = min([min(ekfth) min(odomth) min(measth)]);
axis ([0 max(t_ekf)+4 minaxy-0.1 maxaxy+0.1])
% % Programatically move the Legend
% newPosition = [0.945 0.169 0.01 0.1];
% newUnits = 'normalized';
% set(hL,'Position', newPosition,'Units', newUnits);
xlabel('Time [s]')
ylabel(['Orientation [rad]'])

saveas(fig2, strcat(filename,'wrttime','.jpg'));
saveas(fig2, strcat(filename,'wrttime','.fig'));
saveas(fig2, strcat(filename,'wrttime','.pdf'));
saveas(fig2, strcat(filename,'wrttime','.svg'));