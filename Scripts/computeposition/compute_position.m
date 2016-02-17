function [C_f_x, C_f_y, theta_f] = compute_position (C_i_x, C_i_y, theta_i, nL, nR)
%% Odometry model
%% Inputs: Initial coordinates of the robot (centre position), initial orientation of the robot in radians, nL and nR is rpm of the left and right wheel
%% Outputs: Final coordinates of the robot (centre position), Final orientation of the robot in radians
%% Assumptions:
%% 1. The robot is a rigid body
%% 2. The model represents a differential drive robot
%% 3. There is no slip in the wheel
%% 4. Both the wheels are turning in th forward direction

axle_len = 5 ;
rL = 0.1 ;
rR = 0.1 ;

time_step = 1; %1 s

%compute distance traveled by wheel
SL = time_step*(nL/60)*2*pi*rL ; %linear distance traveled by left wheel in meters
SR = time_step*(nR/60)*2*pi*rR ; %linear distance traveled by right wheel in meters

SL
SR

if (SR > SL)
	r  = (axle_len /2) * ((SL + SR)/(SR - SL) ); %rad_of_travel
	beta_rad = SR / (r + axle_len/2 ); %travel_angle

	r
	beta_rad

	C_f_x = C_i_x - r* ( 1 - cos(beta_rad) ) ;
	C_f_y = C_i_y + r * sin(beta_rad);
	theta_f =  beta_rad/2 + theta_i;
	theta_f_deg = theta_f * 180 / pi

elseif (SL > SR)
	r  = (axle_len /2) * ((SL + SR)/(SL - SR) ); %rad_of_travel
	beta_rad = SR / (r - axle_len/2 ); %travel_angle

	r
	beta_rad

	C_f_x = C_i_x + r*( 1 - cos(beta_rad) ) ;
	C_f_y = C_i_y + r*sin(beta_rad);
	theta_f =  theta_i - beta_rad/2;
	theta_f_deg = theta_f * 180 / pi;
else
	C_f_x = C_i_x + SL*cos(theta_i) ;
	C_f_y = C_i_y + SL*sin(theta_i);
	theta_f =  theta_i;
	theta_f_deg = theta_f * 180 / pi;
end
