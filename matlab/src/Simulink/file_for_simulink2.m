ndoc  = 16;
n_pid = 5;

joint_error_real = importdata(strcat('matlab/data_pwm/',num2str(ndoc),'_motor_error.txt'));joint_states_real = importdata(strcat('matlab/data_pwm/',num2str(ndoc),'_motor_position.txt'));joint_goals = joint_error_real + joint_states_real;

time_real= linspace(0.1, length(joint_goals)*0.1, length(joint_goals));

simu_decimation = 1;
step_time = 0.1;
stop_time = time_real(length(time_real));


input_data = [time_real', joint_goals];

robot = importrobot('archie_description\urdf\manipulator.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

% Con esto únicamente debemos cambiar la variable n_pid para usar otro array de PID
% lo que nos evita tener que andar comentando y/o descomentando código
pid_array(:,:,1) = [
    3 3.5 2.5 2 2 0;
    0.1977    0.6622    0.3440    0.0615    0.0679    0.0026;
    0 0 0 0 0 0
];
pid_array(:,:,2) = [
    2 2.5 1.5 1 1 1;
    0.2451    0.3936    0.2219    0.0530    0.0421    0.0016;             
    0 0 0 0 0 0
];
pid_array(:,:,3) = [
    3.7 4 3 2 2 0.02;
    0.3333    0.4978    0.3138    0.0749    0.0595    0.0009;             
    0 0 0 0 0 0
];
pid_array(:,:,4) = [
    3.9 4.2 3 2 2 0.02;
    0.3333    0.4978    0.3138    0.0749    0.0595    0.0009;             
    0 0 0 0 0 0
];
pid_array(:,:,5) = [
    4.3 4.5 3.4 2 2.5 0.02;
    0.3593    0.5280    0.3341    0.0749    0.0665    0.0002;
    0 0 0 0 0 0
];


k_p = pid_array(1,:,n_pid);
c_p = pid_array(2,:,n_pid);
i_p = pid_array(3,:,n_pid);

out = sim("matlab\src\Simulink\manipulator_pid_torque", 'StopTime', num2str(stop_time));

joint_states_simu = out.joint_states; joint_states_simu = joint_states_simu(1: length(joint_states_simu)-1, :);
joint_error_simu  = (joint_states_simu - joint_goals); % Convertimos a grados

method = "traj";

joint_error_simu  = rad2deg(joint_error_simu);
joint_error_real  = rad2deg(joint_error_real);
joint_goals       = rad2deg(joint_goals); 
joint_states_simu = rad2deg(joint_states_simu);
joint_states_real = rad2deg(joint_states_real);

colors = lines(7);
set(groot, 'defaultFigureWindowState', 'maximized')

figure(); sgtitle(strcat('Error for each joint (PID', num2str(n_pid), ')'));
for i=1:6   
    subplot(3, 2, i); plot(time_real, joint_error_simu(:, i),"LineWidth",1, 'Color', colors(2,:)); hold on;
    subplot(3, 2, i); plot(time_real, joint_error_real(:, i),"LineWidth",1, 'Color', colors(3,:)); 
    legend(strcat("Simu joint ", num2str(i), ", Mean Error: ", num2str(round(mean(abs(joint_error_simu(:, i))), 3))), strcat("Real joint ", num2str(i), ", Mean Error: ", num2str(round(mean(abs(joint_error_real(:, i))), 3))), "Location", "best"); grid minor;
    set(gca,'FontSize',10); xlim([0 time_real(length(time_real))]);
end
xlabel('Time(seconds)', 'FontSize', 12); ylabel('Joint Error (degrees)', 'FontSize', 12); set(gca,'FontSize',10); 
exportgraphics(gcf, strcat(method, "_PID", num2str(n_pid), "_error.png"), "Resolution", 300)


figure();sgtitle(strcat('Input/Output Response (PID', num2str(n_pid), ')'));
for i=1:6   
    subplot(3, 2, i); plot(time_real, joint_goals(:, i),"LineWidth",1, 'Color', colors(1,:)); hold on;
    subplot(3, 2, i); plot(time_real, joint_states_simu(:, i),"LineWidth",1, 'Color', colors(2,:)); hold on;
    subplot(3, 2, i); plot(time_real, joint_states_real(:, i),"LineWidth",1, 'Color', colors(3,:)); 
    legend("Input", "Simu Output", "Real Output", "Location", "best"); grid minor;
    set(gca,'FontSize',10); xlim([0 time_real(length(time_real))]);
end
xlabel('Time(seconds)', 'FontSize', 12); ylabel('Joint Position (degrees)', 'FontSize', 12); set(gca,'FontSize',10); 
exportgraphics(gcf, strcat(method, "_PID", num2str(n_pid), "_in_out.png"), "Resolution", 300)

