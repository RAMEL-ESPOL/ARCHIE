
% joint_torque = out.joint_torques;
% joint_vel    = out.joint_vel;
joint_goals  = out.joint_goals;
joint_states = out.joint_states;
joint_error  = (joint_states - joint_goals)*180/pi;% Convertimos a grados
t = out.time;

n = 7;
method = "step";

colors = lines(6);
set(groot, 'defaultFigureWindowState', 'maximized')


% figure(1); sgtitle(strcat('Torque for each joint (PID', num2str(n), ')'));
% for i=1:6   
%     subplot(3, 2, i); plot(t, joint_torque(:, i),"LineWidth",1, 'Color', colors(i,:)); 
%     legend(strcat("Joint ", num2str(i)), "Location", "best"); grid minor;
%     set(gca,'FontSize',10);        
% end
% xlabel('Time(seconds)', 'FontSize', 12); ylabel('Joint Torques (Nm)', 'FontSize', 12); set(gca,'FontSize',10);
% exportgraphics(gcf, strcat(method, "_PID", num2str(n), "_torque.png"), "Resolution", 300)


figure(); sgtitle(strcat('Error for each joint (PID', num2str(n), ')'));
for i=1:6   
    subplot(3, 2, i); plot(t, joint_error(:, i),"LineWidth",1, 'Color', colors(i,:)); 
    legend(strcat("Joint ", num2str(i), ", Mean Error: ", num2str(mean(abs(joint_error(:, i))))), "Location", "best"); grid minor;
    set(gca,'FontSize',10);    
end
xlabel('Time(seconds)', 'FontSize', 12); ylabel('Joint Error (degrees)', 'FontSize', 12); set(gca,'FontSize',10);
% exportgraphics(gcf, strcat(method, "_PID", num2str(n), "_error.png"), "Resolution", 300)


% figure(3); sgtitle(strcat('Velocity for each joint (PID', num2str(n), ')'));
% for i=1:6   
%     subplot(3, 2, i); plot(t, joint_vel(:, i),"LineWidth",1, 'Color', colors(i,:)); 
%     legend(strcat("Joint ", num2str(i)), "Location", "best"); grid minor;
%     set(gca,'FontSize',10);    
% end
% xlabel('Time(seconds)', 'FontSize', 12); ylabel('Joint Velocity (rad/s)', 'FontSize', 12); set(gca,'FontSize',10);
% exportgraphics(gcf, strcat(method, "_PID", num2str(n), "_vel.png"), "Resolution", 300)


figure();sgtitle(strcat('Input/Output Response (PID', num2str(n), ')'));
for i=1:6   
    subplot(3, 2, i); plot(t, joint_goals(:, i),"LineWidth",1, 'Color', colors(1,:)); hold on;
    subplot(3, 2, i); plot(t, joint_states(:, i),"LineWidth",1, 'Color', colors(2,:)); 
    legend("Input", "Output", "Location", "best"); grid minor;
    set(gca,'FontSize',10);    
end
xlabel('Time(seconds)', 'FontSize', 12); ylabel('Joint Position (degrees)', 'FontSize', 12); set(gca,'FontSize',10);
% exportgraphics(gcf, strcat(method, "_PID", num2str(n), "_in_out.png"), "Resolution", 300)