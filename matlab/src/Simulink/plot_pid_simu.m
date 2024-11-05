
joint_torque = out.joint_torques;
joint_vel    = out.joint_vel;
joint_goals  = out.joint_goals;
joint_states = out.joint_states;
joint_error  = (joint_states - joint_goals)*180/pi;% Convertimos a grados
t = out.time;


colors = lines(6);
set(groot, 'defaultFigureWindowState', 'maximized')


% figure(1); sgtitle('Torque for each joint');
% for i=1:6   
%     subplot(3, 2, i); plot(t, joint_torque(:, i),"LineWidth",1, 'Color', colors(i,:)); 
%     legend(strcat("Joint ", num2str(i)), "Location", "best"); grid minor;
%     set(gca,'FontSize',10);        
% end
% xlabel('Time(seconds)', 'FontSize', 12); ylabel('Joint Torques (Nm)', 'FontSize', 12); set(gca,'FontSize',10);
% exportgraphics(gcf, "step_PD1_torque.png", "Resolution", 300)


figure(2); sgtitle('Error for each joint');
for i=1:6   
    subplot(3, 2, i); plot(t, joint_error(:, i),"LineWidth",1, 'Color', colors(i,:)); 
    legend(strcat("Joint ", num2str(i), ", Mean Error: ", num2str(mean(abs(joint_error(:, i))))), "Location", "best"); grid minor;
    set(gca,'FontSize',10);    
end
xlabel('Time(seconds)', 'FontSize', 12); ylabel('Joint Error (degrees)', 'FontSize', 12); set(gca,'FontSize',10);
exportgraphics(gcf, "step_PD1_error.png", "Resolution", 300)


% figure(3); sgtitle('Velocity for each joint');
% for i=1:6   
%     subplot(3, 2, i); plot(t, joint_vel(:, i),"LineWidth",1, 'Color', colors(i,:)); 
%     legend(strcat("Joint ", num2str(i)), "Location", "best"); grid minor;
%     set(gca,'FontSize',10);    
% end
% xlabel('Time(seconds)', 'FontSize', 12); ylabel('Joint Velocity (rad/s)', 'FontSize', 12); set(gca,'FontSize',10);
% exportgraphics(gcf, "step_PD1_vel.png", "Resolution", 300)


figure(4); sgtitle('Step Response');
for i=1:6   
    subplot(3, 2, i); plot(t, joint_goals(:, i),"LineWidth",1, 'Color', colors(1,:)); hold on;
    subplot(3, 2, i); plot(t, joint_states(:, i),"LineWidth",1, 'Color', colors(2,:)); 
    legend("Step Input", "Step Output", "Location", "best"); grid minor;
    set(gca,'FontSize',10);    
end
xlabel('Time(seconds)', 'FontSize', 12); ylabel('Joint Position (degrees)', 'FontSize', 12); set(gca,'FontSize',10);
exportgraphics(gcf, "step_PD1_in_out.png", "Resolution", 300)