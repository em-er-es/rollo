function [x_cc,E_cc]= rollo_ekf(fstate,x_pp,u,Jfunc,Jh,E_pp,hmeas,z,Q,R)
% Original code: By Yi Cao at Cranfield University, 02/01/2008
% Modified by: Rabbia Asgahr specific to Odometry, 15/02/2016
% EKF   Extended Kalman Filter for nonlinear dynamic systems
% [x, P] = ekf(fstate,x,u,Jf,Jh,P,hmeas,z,Q,R) returns state estimate, x and state covariance, P 
% for nonlinear dynamic system:
%           x_k = f(x_k-1, u_k-1) + w_k
%           z_k   = h(x_k) + v_k
% where w ~ N(0,Q) meaning w is gaussian noise with covariance Q
%       v ~ N(0,R) meaning v is gaussian noise with covariance R
% Inputs:   fstate: function handle for f(x_k-1, u_k-1)
%           x_pp: "a priori" state estimate, x_k-1|k-1 (p refers to k-1)  
%           u: control input, u_k-1
%           Jfunc: function handle for Jacobian matrix with the partial derivatives of f(x_k-1,u_k-1)
%           Jh: Jacobian matrix with partial derivatives of h(x_k)
%           E_pp: "a priori" estimated state covariance, E_k-1|k-1 (p refers to k-1) 
%           hmeas: function handle for h(x)
%           z: current measurement
%           Q: process noise covariance 
%           R: measurement noise covariance
% Output:   x_cc: "a posteriori" state estimate, x_k|k (c refers to k) 
%           E_cc: "a posteriori" state covariance, E_k|k (c refers to k) 
%

% PREDICTION UPDATE
%nonlinear update and linearization at current state
x_cp = fstate(x_pp,u); %state prediction, x_k|k-1
Jf = Jfunc(x_pp,u);

%partial covariance update
E_cp = Jf*E_pp*Jf' + Q;  %E_k|k-1              

% INNOVATION UPDATE
%nonlinear measurement and linearization   
z_estimate = hmeas(x_cp);

P12=E_cp*Jh'; %cross covariance
S_inv = inv(Jh*P12+R);
% M_inv = matrix3by3_inverse(H*P12+R)
H = P12*S_inv;       %Kalman filter gain, H_k


x_cc = x_cp + H*(z-z_estimate);     %state estimate, x_k|k
E_cc = E_cp - H*P12';               %state covariance matrix, E_k|k
