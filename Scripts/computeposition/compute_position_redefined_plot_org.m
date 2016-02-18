function [C_f_x, C_f_y, theeta_f] = compute_position (C_i_x, C_i_y, theeta_i, nL, nR)
%% Odometery mode
%% inputs: Initial coordinates of the robot (centre position), initial orientation of the robot in radians, nL and nR is rpm of the left and right wheel 
%% outputs: Final coordinates of the robot (centre position), Final orientation of the robot in radians
%% Assumptions: 
%% 1. the robot is a rigid body
%% 2. the model represents a differential drive robot
%% 3. there is no slip in the wheel
%% 4. both the wheels are turning in th forward direction

axle_len = 5 ;
rL = 0.1 ;
rR = 0.1 ;

time_step = 1; %1 s
%time_step = 10; %1 s

%compute distance traveled by wheel
SL = time_step*(nL/60)*2*pi*rL ; %linear distance traveled by left wheel in meters
SR = time_step*(nR/60)*2*pi*rR ; %linear distance traveled by right wheel in meters

SL
SR

 %now repeating so we have the same equation for all three conditions
 for i = 1:100
 
    beta_rad = (SL - SR) / 2;
    del_S = (SL+SR)/2;
%    beta_rad
    
    C_f_x = C_i_x + del_S* cos(-beta_rad/2 + theeta_i) ; 
    C_f_y = C_i_y + del_S* sin(-beta_rad/2 + theeta_i);    
   theeta_f =  theeta_i - beta_rad;
   theeta_f_deg = theeta_f * 180 / pi;
  
   C_x(i) = C_f_x ;
   C_y(i) = C_f_y ;
   
   theeta_i = theeta_f;
   C_i_x = C_f_x;
   C_i_y = C_f_y;
   
 end
 
 
 plot(C_x, C_y);
 axis equal;
 axis ([-5 5 -5 5])
