%analyze logs to estimate control input in absence of encoders
%READ FROM LOGFILE
[x_log,y_log,theta_log] = extract_position_edit2('Test1v6AB.log');
plot (x_log,y_log);
p = polyfit(x_log,y_log,1);
hold on
plot(x_log, polyval(p, x_log))
% once we have line of best fit, find delta S/
% delta S = S/total time
% S = diatnce between x1,y1 anf xfinal,yfinal
% for delta theta?
% atan(yfinal-y1,xfinal-x1)