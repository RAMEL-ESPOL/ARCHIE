ndoc  = 29;
n_pid = 3;
frec  = 20;
method = "step";
simu_decimation = 1;
step_time = 1/frec;

cart_error_real  = importdata(strcat('matlab/data_pwm/',num2str(ndoc),'_motor_error.txt'));
cart_states_real = importdata(strcat('matlab/data_pwm/',num2str(ndoc),'_motor_position.txt'));
cart_goals = cart_error_real + cart_states_real;

cart_torque_real = importdata(strcat('matlab/data_pwm/',num2str(ndoc),'_motor_effort.txt'));

time_real= linspace(1/frec, length(cart_goals)/frec, length(cart_goals));
input_data = [time_real', cart_goals];
stop_time = time_real(length(time_real));

robot = importrobot('archie_description\urdf\manipulator.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

% Con esto únicamente debemos cambiar la variable n_pid para usar otro array de PID
% lo que nos evita tener que andar comentando y/o descomentando código
pid_array(:,:,1) = [
    50 50 50;
    1 1 1;
    0 0 0 
];
pid_array(:,:,2) = [
    70 70 70;
    4 4 4;
    0 0 0 
];
pid_array(:,:,3) = [
    60 60 60;
    4 4 4;
    0 0 0 
];
k_p = pid_array(1,:,n_pid);
k_d = pid_array(2,:,n_pid);
k_i = pid_array(3,:,n_pid);

out = sim("matlab\src\Simulink\cartesian_manipulator_pid_torque", 'StopTime', num2str(stop_time));


% colors = lines(7);
% set(groot, 'defaultFigureWindowState', 'maximized')

% cart = ["X" "Y" "Z"];

% figure(); sgtitle(strcat('Error for X, Y, Z (PID', num2str(n_pid), ') @', num2str(frec), 'Hz'));
% for i=1:3   
%     subplot(3, 1, i); plot(time_real, cart_error_real(:, i),"LineWidth",1, 'Color', colors(3,:)); 
%     legend(strcat("Real pos ", num2str(i), ", Mean Error: ", num2str(round(mean(abs(cart_error_real(:, i))), 3))), "Location", "best"); grid minor;
%     set(gca,'FontSize',10); xlim([0 time_real(length(time_real))]);
%     xlabel('Time(seconds)', 'FontSize', 12); ylabel(strcat(cart(i), ' Error (m)'), 'FontSize', 12); set(gca,'FontSize',10); 
% end
% exportgraphics(gcf, strcat(num2str(frec),"_",method, "_PID", num2str(n_pid), "_f",num2str(frec), "_ndoc", num2str(ndoc), "_error.png"), "Resolution", 300)


% figure();sgtitle(strcat('Input/Output Response (PID', num2str(n_pid), ') @', num2str(frec), 'Hz'));
% for i=1:3   
%     subplot(3, 1, i); plot(time_real, cart_goals(:, i),"LineWidth",1, 'Color', colors(1,:)); hold on;
%     subplot(3, 1, i); plot(time_real, cart_states_real(:, i),"LineWidth",1, 'Color', colors(3,:)); 
%     legend("Input", "Real Output", "Location", "best"); grid minor;
%     set(gca,'FontSize',10); xlim([0 time_real(length(time_real))]);
%     xlabel('Time(seconds)', 'FontSize', 12); ylabel(strcat(cart(i), ' Position (m)'), 'FontSize', 12); set(gca,'FontSize',10); 
% end
% exportgraphics(gcf, strcat(num2str(frec),"_",method, "_PID", num2str(n_pid), "_f",num2str(frec), "_ndoc", num2str(ndoc), "_in_out.png"), "Resolution", 300)

% figure(); sgtitle(strcat("Torque for each cart (PID", num2str(n_pid), ') @', num2str(frec), 'Hz'));
% for i=1:6   
%     subplot(3, 2, i); plot(time_real, cart_torque_real(:, i),"LineWidth",1, 'Color', colors(3,:)); 
%     legend(strcat("Real torque", num2str(i)), "Location", "best"); grid minor;
%     set(gca,'FontSize',10); xlim([0 time_real(length(time_real))]);
% end
% xlabel('Time(seconds)', 'FontSize', 12); ylabel('Torques (Nm)', 'FontSize', 12); set(gca,'FontSize',10);
% exportgraphics(gcf, strcat(num2str(frec),"_",method, "_PID", num2str(n_pid), "_f",num2str(frec), "_ndoc", num2str(ndoc), "_torque.png"), "Resolution", 300)

