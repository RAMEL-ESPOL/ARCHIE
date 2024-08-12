robot = importrobot('archie_description\urdf\manipulator.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

ndoc = 11;
eff = importdata(strcat('matlab/data_pwm/',num2str(ndoc),'_motor_effort.txt'  ));
err = importdata(strcat('matlab/data_pwm/',num2str(ndoc),'_motor_error.txt'   ));
pos = importdata(strcat('matlab/data_pwm/',num2str(ndoc),'_motor_position.txt'));
vel = importdata(strcat('matlab/data_pwm/',num2str(ndoc),'_motor_velocity.txt'));

colors = lines(6); % Define a set of colors

% Plot Motor Positions
figure(1)
sgtitle('Motor Positions with Vel Control start in other position', 'FontSize', 15); % Super title for the entire figure
for i = 1:6
    subplot(3,2,i)
    grid on
    grid minor
    hold on
    plot(pos(:,i), 'LineWidth', 2, 'Color', colors(i,:))
    legend(strcat("Joint ", num2str(i-1), " (rad)"))
end
xlabel('Iterations', 'FontSize', 10); % Add a single x-axis label
ylabel('Position (rad)', 'FontSize', 10); % Add a single y-axis label

% Plot Motor Velocities
figure(2)
sgtitle('Motor Velocities with Vel Control start in other position', 'FontSize', 15); % Super title for the entire figure
for i = 1:6
    subplot(3,2,i)
    grid on
    grid minor
    hold on
    plot(vel(:,i), 'LineWidth', 2, 'Color', colors(i,:))
    legend(strcat("Joint ", num2str(i-1), " (RPM)"))
end
xlabel('Iterations', 'FontSize', 10); % Add a single x-axis label
ylabel('Velocity (RPM)', 'FontSize', 10); % Add a single y-axis label

% Plot Motor Effort
figure(3)
sgtitle('Motor Effort with Vel Control start in other position', 'FontSize', 15); % Super title for the entire figure
for i = 1:6
    subplot(3,2,i)
    grid on
    grid minor
    hold on
    plot(eff(:,i), 'LineWidth', 2, 'Color', colors(i,:))
    legend(strcat("Joint ", num2str(i-1), " (Nm)"))
end
xlabel('Iterations', 'FontSize', 10); % Add a single x-axis label
ylabel('Effort (Nm)', 'FontSize', 10); % Add a single y-axis label

% Plot Motor Position Error
figure(4)
sgtitle('Motor Position Error with Vel Control start in other position', 'FontSize', 15); % Super title for the entire figure
for i = 1:6
    subplot(3,2,i)
    grid on
    grid minor
    hold on
    plot(err(:,i), 'LineWidth', 2, 'Color', colors(i,:))
    legend(strcat("Joint ", num2str(i-1), " (rad)"))
end
xlabel('Iterations', 'FontSize', 10); % Add a single x-axis label
ylabel('Position Error (rad)', 'FontSize', 10); % Add a single y-axis label

disp("Matriz de masa:")
disp(massMatrix(robot, [0 0 0 0 0 0]));
