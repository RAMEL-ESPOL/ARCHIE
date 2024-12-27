ndoc  = 17;
n_pid = 5;
frec  = 30;

method = "traj";

joint_error_simu  = rad2deg(joint_error_simu);
joint_error_dyna  = rad2deg(joint_error_dyna);
joint_error_real  = rad2deg(joint_error_real);
joint_goals       = rad2deg(joint_goals); 
joint_states_simu = rad2deg(joint_states_simu);
joint_states_real = rad2deg(joint_states_real);

colors = lines(7);
set(groot, 'defaultFigureWindowState', 'maximized')

figure(); sgtitle(strcat('Error for each joint (PID', num2str(n_pid), ')'));
for i=1:6   
    subplot(3, 2, i); plot(time_real, joint_error_simu(:, i),"LineWidth",1, 'Color', colors(2,:)); hold on;
    subplot(3, 2, i); plot(time_real, joint_error_dyna(:, i),"LineWidth",1, 'Color', colors(3,:)); hold on;
    subplot(3, 2, i); plot(time_real, joint_error_real(:, i),"LineWidth",1, 'Color', colors(4,:)); 
    legend(strcat("Simu joint ", num2str(i), ", Mean Error: ", num2str(round(mean(abs(joint_error_simu(:, i))), 3))), strcat("New simu joint ", num2str(i), ", Mean Error: ", num2str(round(mean(abs(joint_error_dyna(:, i))), 3))), strcat("Real joint ", num2str(i), ", Mean Error: ", num2str(round(mean(abs(joint_error_real(:, i))), 3))), "Location", "best"); grid minor;
    set(gca,'FontSize',10); xlim([0 time_real(length(time_real))]);
end
xlabel('Time(seconds)', 'FontSize', 12); ylabel('Joint Error (degrees)', 'FontSize', 12); set(gca,'FontSize',10); 
exportgraphics(gcf,strcat(num2str(frec),"_",method, "_PID", num2str(n_pid), "_error.png"), "Resolution", 300)


figure();sgtitle(strcat('Input/Output Response (PID', num2str(n_pid), ')'));
for i=1:6   
    subplot(3, 2, i); plot(time_real, joint_goals(:, i),"LineWidth",1, 'Color', colors(1,:)); hold on;
    subplot(3, 2, i); plot(time_real, joint_states_simu(:, i),"LineWidth",1, 'Color', colors(2,:)); hold on;
    subplot(3, 2, i); plot(time_real, joint_states_dyna(:, i),"LineWidth",1, 'Color', colors(3,:)); hold on;
    subplot(3, 2, i); plot(time_real, joint_states_real(:, i),"LineWidth",1, 'Color', colors(4,:)); 
    legend("Input", "Simu Output", "New simu output", "Real Output", "Location", "best"); grid minor;
    set(gca,'FontSize',10); xlim([0 time_real(length(time_real))]);
end
xlabel('Time(seconds)', 'FontSize', 12); ylabel('Joint Position (degrees)', 'FontSize', 12); set(gca,'FontSize',10); 
exportgraphics(gcf,strcat(num2str(frec),"_",method, "_PID", num2str(n_pid), "_in_out.png"), "Resolution", 300)

