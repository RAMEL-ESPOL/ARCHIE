

joint_states_simu = out.joint_states; joint_states_simu = joint_states_simu(1: length(joint_states_simu)-1, :);
joint_error_simu  = (joint_states_simu - joint_goals)*180/pi;% Convertimos a grados


t = time_real;

n = 1;
method = "traj";

colors = lines(7);
set(groot, 'defaultFigureWindowState', 'maximized')

figure(); sgtitle(strcat('Error for each joint (PID', num2str(n), ')'));
for i=1:6   
    subplot(3, 2, i); plot(t, joint_error_simu(:, i),"LineWidth",1, 'Color', colors(i,:)); hold on;
    subplot(3, 2, i); plot(t, joint_error_real(:, i),"LineWidth",1, 'Color', colors(i+1,:)); 
    legend(strcat("Simu joint ", num2str(i), ", Mean Error: ", num2str(round(mean(abs(joint_error_simu(:, i))), 3))), strcat("Real joint ", num2str(i), ", Mean Error: ", num2str(round(mean(abs(joint_error_real(:, i))), 3))), "Location", "best"); grid minor;
    set(gca,'FontSize',10);    
end
xlabel('Time(seconds)', 'FontSize', 12); ylabel('Joint Error (degrees)', 'FontSize', 12); set(gca,'FontSize',10);
exportgraphics(gcf, strcat(method, "_PID", num2str(n), "_error.png"), "Resolution", 300)


figure();sgtitle(strcat('Input/Output Response (PID', num2str(n), ')'));
for i=1:6   
    subplot(3, 2, i); plot(t, joint_goals(:, i),"LineWidth",1, 'Color', colors(1,:)); hold on;
    subplot(3, 2, i); plot(t, joint_states_simu(:, i),"LineWidth",1, 'Color', colors(2,:)); hold on;
    subplot(3, 2, i); plot(t, joint_states_real(:, i),"LineWidth",1, 'Color', colors(3,:)); 
    legend("Input", "Simu Output", "Real Output", "Location", "best"); grid minor;
    set(gca,'FontSize',10);    
end
xlabel('Time(seconds)', 'FontSize', 12); ylabel('Joint Position (degrees)', 'FontSize', 12); set(gca,'FontSize',10);
exportgraphics(gcf, strcat(method, "_PID", num2str(n), "_in_out.png"), "Resolution", 300)
