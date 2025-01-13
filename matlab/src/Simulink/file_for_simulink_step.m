
simu_decimation = 1;
frec  = 100;
step_time = 1/frec;
stop_time = 3;
n_pid = 1;

joint_goals = zeros(stop_time/step_time, 6);
joint_goals(stop_time/step_time/2:stop_time/step_time, :) = ones(stop_time/step_time/2 + 1, 6).*[1.57 1.57/2 -1.57/2 1.57/3 1.57 1.57];
joint_goals_step = [1.57 1.57/2 -1.57/2 1.57/3 1.57 1.57];
robot = importrobot('archie_description\urdf\manipulator.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

time_real= linspace(1/frec, length(joint_goals)/frec, length(joint_goals));

simu_decimation = 1;
step_time = 1/frec;
stop_time = time_real(length(time_real));

input_data = [time_real', joint_goals];


% Con esto únicamente debemos cambiar la variable ndoc para usar otro array de PID
% lo que nos evita tener que andar comentando y/o descomentando código

pid_array(:,:,1) = [
    0 0 0 0 0 0
    0 0 0 0 0 0;
    0 0 0 0 0 0
];



k_p = pid_array(1,:,n_pid);
c_p = pid_array(2,:,n_pid);
i_p = pid_array(3,:,n_pid);


out = sim("matlab\src\Simulink\manipulator_pid_torque_2019", 'StopTime', num2str(stop_time));

joint_states_simu = out.joint_states; joint_states_simu = joint_states_simu(1: length(joint_states_simu)-1, :);
joint_error_simu  = (joint_goals - joint_states_simu);

joint_error_simu  = rad2deg(joint_error_simu);
joint_goals       = rad2deg(joint_goals); 
joint_states_simu = rad2deg(joint_states_simu);
% time_real = out.time(1: length(out.time)-1, :);
colors = lines(7);
set(groot, 'defaultFigureWindowState', 'maximized')

% figure(); sgtitle(strcat('Error for each joint (PID', num2str(n_pid), ') @', num2str(frec), 'Hz'));
% for i=1:6   
%     subplot(3, 2, i); plot(time_real, joint_error_simu(:, i),"LineWidth",1, 'Color', colors(2,:)); hold on;
%     legend(strcat("Simu joint ", num2str(i), ", Mean Error: ", num2str(round(mean(abs(joint_error_simu(:, i))), 3))), "Location", "best"); grid minor;
%     set(gca,'FontSize',10); xlim([0 time_real(length(time_real))]);
% end
% xlabel('Time(seconds)', 'FontSize', 12); ylabel('Joint Error (degrees)', 'FontSize', 12); set(gca,'FontSize',10); 


figure();sgtitle(strcat('Input/Output Response (Control PD) @', num2str(frec), 'Hz'));
for i=1:6   
    subplot(3, 2, i); plot(time_real, joint_goals(:, i),"LineWidth",1, 'Color', colors(1,:)); hold on;
    subplot(3, 2, i); plot(time_real, joint_states_simu(:, i),"LineWidth",1, 'Color', colors(2,:)); hold on;
    legend("Input", "Simu Output", "Location", "best"); grid minor;
    set(gca,'FontSize',10); xlim([0 time_real(length(time_real))]);
end
xlabel('Time(seconds)', 'FontSize', 12); ylabel('Joint Position (degrees)', 'FontSize', 12); set(gca,'FontSize',10); 


% figure();sgtitle(strcat('Input/Output Response (PID', num2str(n_pid), ') @', num2str(frec), 'Hz'));
% i = 1;
% plot(time_real, joint_goals(:, i),"LineWidth",1, 'Color', colors(1,:)); hold on;
% plot(time_real, joint_states_simu(:, i),"LineWidth",1, 'Color', colors(2,:)); hold on;
% legend("Input", "Simu Output", "Location", "best"); grid minor;
% set(gca,'FontSize',10); xlim([0 time_real(length(time_real))]);
% 
% xlabel('Time(seconds)', 'FontSize', 12); ylabel('Joint Position (degrees)', 'FontSize', 12); set(gca,'FontSize',10); 

