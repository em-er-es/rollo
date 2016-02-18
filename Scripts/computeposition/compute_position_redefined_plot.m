function [C_f_x, C_f_y, theta_f] = compute_position (C_i_x, C_i_y, theta_i, nL, nR, t)
%% Odometery model
%% Inputs: Initial coordinates of the robot (centre position), initial orientation of the robot in radians, nL and nR is rpm of the left and right wheel 
%% Outputs: Final coordinates of the robot (centre position), Final orientation of the robot in radians
%% Assumptions: 
%% 1. The robot is a rigid body
%% 2. The model represents a differential drive robot
%% 3. There is no slip in the wheel
%% 4. Both the wheels are turning in th forward direction

axle_len = 0.205; % [m]
rL = 0.1; % [m]
rR = 0.1; % [m]
t % [s]
steps = 100;

%compute distance traveled by wheel
% SL = t * (nL / 60) * 2 * pi * rL; %linear distance traveled by left wheel in meters
% SR = t * (nR / 60) * 2 * pi * rR; %linear distance traveled by right wheel in meters
td = t / steps
SL = td * (nL / 60) * 2 * pi * rL; %linear distance traveled by left wheel in meters
SR = td * (nR / 60) * 2 * pi * rR; %linear distance traveled by right wheel in meters

C_i_x, C_i_y, SL, SR, theta_i


hold off;
plot(C_i_x, C_i_y, 'o');
hold on;

%now repeating so we have the same equation for all three conditions
for i = 1:steps

	beta_rad = (SL - SR) / 2;
	del_S = (SL + SR) / 2;

	C_f_x = C_i_x + del_S* cos(-beta_rad/2 + theta_i);
	C_f_y = C_i_y + del_S* sin(-beta_rad/2 + theta_i);

	% del_S
	% beta_rad
	% theta_i
	% (-beta_rad/2 + theta_i)

	theta_f =  theta_i - beta_rad;
	theta_f_deg = theta_f * 180 / pi;

	C_x(i) = C_f_x;
	C_y(i) = C_f_y;

	theta_i = theta_f;
	C_i_x = C_f_x;
	C_i_y = C_f_y;

end

theta_f_deg = mod(theta_f_deg, 360)
theta_f = mod(theta_f, 2*pi)
% theta_f_deg

plot(C_f_x, C_f_y, 'x');
plot(C_x, C_y);
axis equal;
axx = ceil((max(C_x) - min(C_x)) / 5) * 5;
axy = ceil((max(C_y) - min(C_y)) / 5) * 5;
% axis ([-axx axx -axy axy])
grid on;
