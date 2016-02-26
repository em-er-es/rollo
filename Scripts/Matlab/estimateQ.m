%% estimate Q, process covariance matrix

% read log with v 12%
[x_log,y_log,theta_log] = extract_position_edit2('Test10v12BC.log');

% for v 12%, control input is as follows in the model
u = [ 0.072433; 0.00008584 ];

N = 1500;
error_x_sq = zeros(1,N);
error_y_sq = zeros(1,N);
error_theta_sq = zeros(1,N);

for i = 1:N
    x_pp = [x_log(i); y_log(i); theta_log(i)];
    
    f_xu = [x_pp(1) + u(1)*cos(x_pp(3) - (u(2)/2)); x_pp(2) + u(1)*sin(x_pp(3) - (u(2)/2)); x_pp(3) - u(2)];  % nonlinear state equations, f(x_k-1,u_k-1)

    error = [f_xu(1) - x_log(i+1);  f_xu(2) - y_log(i+1); f_xu(3) - theta_log(i+1)];
    
    error_x_sq(i) = error(1);
    error_y_sq(i) = error(2);
    error_theta_sq(i) = error(3);
    
end

mean_x_error = mean(error_x_sq)
std_x_error = std(error_x_sq)
    
mean_y_error = mean(error_y_sq)
std_y_error = std(error_y_sq)

mean_theta_error = mean(error_theta_sq)
std_theta_error = std(error_theta_sq)