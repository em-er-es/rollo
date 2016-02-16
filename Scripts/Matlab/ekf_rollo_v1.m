% EKF   Extended Kalman Filter for nonlinear dynamic systems
% [x, P] = ekf(f,x,P,h,z,Q,R) returns state estimate, x and state covariance, P 
% for nonlinear dynamic system:
%           x_k+1 = f(x_k, u_k) + w_k
%           z_k   = h(x_k) + v_k
% where w ~ N(0,Q) meaning w is gaussian noise with covariance Q
%       v ~ N(0,R) meaning v is gaussian noise with covariance R
% Inputs:   f: function handle for f(x)
%           x: "a priori" state estimate
%           P: "a priori" estimated state covariance
%           h: fanction handle for h(x)
%           z: current measurement
%           Q: process noise covariance 
%           R: measurement noise covariance
% Output:   x: "a posteriori" state estimate
%           P: "a posteriori" state covariance

% initialization of the system

n=3;      %number of states

%however Q will need to be modeled in actual system
q=0.1;    %std of process noise  
r=0.1;    %std of measurement noise
Q=q^2*eye(n); % covariance of process
R=r^2*eye(n); % covariance of measurement  



%READ FROM LOGFILE
[x_log,y_log,theta_log] = extract_position_edit2('Test1v6AB.log', 1000, 1500, 1);
%assuming straight line motion
deltaS = ( x_log(length(x_log)) - x_log(1) )/ length(x_log);
u = [deltaS ; 0];                   %control input [delta S, theta], assumed constant in this example


s=[x_log(1);y_log(1);theta_log(1)]; % initial state
x=s+q*randn(3,1);   % initial state with noise(noise is optional as we will read inital state accurately from motion capture)
  

f=@(x,u)[x(1) + u(1)*cos(x(3) - (u(2)/2)); x(2) + u(1)*sin(x(3) - (u(2)/2)); x(3) - u(2)];  % nonlinear state equations, f(x_k-1,u_k-1)
h=@(x)[x(1); x(2); x(3)];                  % measurement equation, h(x_k)

Jf =@ (x,u) [1, 0, - u(1)*sin(x(3) - (u(2)/2)); 0, 1, u(1)*cos(x(3) - (u(2)/2)); 0, 0, 1]; %Jacobian matrix with the partial derivatives of f(x_k-1,u_k-1) w.r.t x
Jh = eye(n); %Jacobian matrix with the partial derivatives of h(x_k) w.r.t x, identity in this case


E = eye(n);            % initial state covraiance

N=100;                    % total dynamic steps
xV = zeros(n,N);          % allocate memory for state
zV = zeros(n,N);          % allocate memory for measurement

for k=1:N
  z = [x_log(k+1);y_log(k+1);theta_log(k+1)];      % measurments from sensor
  zV(:,k)  = z;                             % save measurment
  [x, E] = rollo_ekf(f,x,u,Jf,Jh,E,h,z,Q,R);    % EKF 
  xV(:,k) = x;                            % save estimate
end
figure

 % plot results
for k=1:3                                
  subplot(3,1,k)
  plot(1:N, xV(k,:), '-', 1:N, zV(k,:), '--')
  title(['x state [',num2str(k),']'])
  legend('state estimate', 'measurement')
end
