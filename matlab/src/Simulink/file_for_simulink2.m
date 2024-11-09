ndoc = 16;
n = 5;

joint_error_real = importdata(strcat('matlab/data_pwm/',num2str(ndoc),'_motor_error.txt'));joint_states_real = importdata(strcat('matlab/data_pwm/',num2str(ndoc),'_motor_position.txt'));joint_goals = joint_error_real + joint_states_real;

time_real= [];

for i=1:length(joint_goals)
    time_real = [time_real, i*0.1];
end

simu_decimation = 10;
step_time = 0.01;
stop_time = time_real(length(time_real));


input_data = [time_real', joint_goals];

robot = importrobot('archie_description\urdf\manipulator.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];




% PID1
k_p = [3 3.5 2.5 2 2 0];
c_p = [0.1977    0.6622    0.3440    0.0615    0.0679    0.0026];
i_p = [0 0 0 0 0 0];


% PID2
k_p = [2 2.5 1.5 1 1 1];
c_p = [0.2451    0.3936    0.2219    0.0530    0.0421    0.0016];
i_p = [0 0 0 0 0 0];

% PID3
k_p = [3.7 4 3 2 2 0.02];
c_p = [0.3333    0.4978    0.3138    0.0749    0.0595    0.0009];
i_p = [0 0 0 0 0 0];

% PID4
k_p = [3.9 4.2 3 2 2 0.02];
c_p = [0.3333    0.4978    0.3138    0.0749    0.0595    0.0009];
i_p = [0 0 0 0 0 0];

% PID5
k_p = [4.3 4.5 3.4 2 2.5 0.02];
c_p = [0.3593    0.5280    0.3341    0.0749    0.0665    0.0002];
i_p = [0 0 0 0 0 0];





out = sim("matlab\src\Simulink\manipulator_pid_torque", 'StopTime', num2str(stop_time));

joint_states_simu = out.joint_states; joint_states_simu = joint_states_simu(1: length(joint_states_simu)-1, :);
joint_error_simu  = (joint_states_simu - joint_goals)*180/pi;% Convertimos a grados


t = time_real;

method = "traj";

colors = lines(7);
set(groot, 'defaultFigureWindowState', 'maximized')

figure(); sgtitle(strcat('Error for each joint (PID', num2str(n), ')'));
for i=1:6   
    subplot(3, 2, i); plot(t, joint_error_simu(:, i),"LineWidth",1, 'Color', colors(i,:)); hold on;
    subplot(3, 2, i); plot(t, joint_error_real(:, i),"LineWidth",1, 'Color', colors(i+1,:)); 
    legend(strcat("Simu joint ", num2str(i), ", Mean Error: ", num2str(round(mean(abs(joint_error_simu(:, i))), 3))), strcat("Real joint ", num2str(i), ", Mean Error: ", num2str(round(mean(abs(joint_error_real(:, i))), 3))), "Location", "best"); grid minor;
    set(gca,'FontSize',10); xlim([0 time_real(length(time_real))]);
end
xlabel('Time(seconds)', 'FontSize', 12); ylabel('Joint Error (degrees)', 'FontSize', 12); set(gca,'FontSize',10); 
exportgraphics(gcf, strcat(method, "_PID", num2str(n), "_error.png"), "Resolution", 300)


figure();sgtitle(strcat('Input/Output Response (PID', num2str(n), ')'));
for i=1:6   
    subplot(3, 2, i); plot(t, joint_goals(:, i),"LineWidth",1, 'Color', colors(1,:)); hold on;
    subplot(3, 2, i); plot(t, joint_states_simu(:, i),"LineWidth",1, 'Color', colors(2,:)); hold on;
    subplot(3, 2, i); plot(t, joint_states_real(:, i),"LineWidth",1, 'Color', colors(3,:)); 
    legend("Input", "Simu Output", "Real Output", "Location", "best"); grid minor;
    set(gca,'FontSize',10); xlim([0 time_real(length(time_real))]);
end
xlabel('Time(seconds)', 'FontSize', 12); ylabel('Joint Position (degrees)', 'FontSize', 12); set(gca,'FontSize',10); 
exportgraphics(gcf, strcat(method, "_PID", num2str(n), "_in_out.png"), "Resolution", 300)

