robot = importrobot('archie_description\urdf\manipulator.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

ndoc = 11;

motor_err = importdata(strcat('matlab/data_pwm/',num2str(ndoc),'_motor_error.txt'   ));
motor_pos = importdata(strcat('matlab/data_pwm/',num2str(ndoc),'_motor_position.txt'));

goals_pos = motor_err + motor_pos;

cart_motor = [];
cart_goals = [];

for i=1:length(motor_pos)
    tform_motor = getTransform(robot, motor_pos(i, : ), 'link_6'); 
    pos_motor = tform2trvec(tform_motor);
    cart_motor = [cart_motor; pos_motor];
    
    tform_goals = getTransform(robot, goals_pos(i, : ), 'link_6'); 
    pos_goals = tform2trvec(tform_goals);
    cart_goals = [cart_goals; pos_goals];

end


cart_error = (cart_goals - cart_motor);
colors = lines(3); % Define a set of colors

for i=1:3
    figure(1);
    subplot(3,1,i)
    plot(cart_motor(:,i),"LineWidth",2, 'Color', colors(i,:))
    grid on
    grid minor

    if i == 1
        title("Cartesian position of the end-effector (motor)")
        xlabel("Iteration")
        ylabel("X (m)")
    elseif i == 2
        xlabel("Iteration")
        ylabel("Y (m)")
    else
        xlabel("Iteration")
        ylabel("Z (m)")
    end

end

for i=1:3
    figure(2);
    subplot(3,1,i)
    plot(cart_goals(:,i),"LineWidth",2, 'Color', colors(i,:))
    grid on
    grid minor

    if i == 1
        title("Cartesian position of the end-effector (goals)")
        xlabel("Iteration")
        ylabel("X (m)")
    elseif i == 2
        xlabel("Iteration")
        ylabel("Y (m)")
    else
        xlabel("Iteration")
        ylabel("Z (m)")
    end

end

for i=1:3
    figure(3);
    subplot(3,1,i)
    plot(cart_error(:,i),"LineWidth",2, 'Color', colors(i,:))
    grid on
    grid minor

    if i == 1
        title("Cartesian position of the end-effector (error)")
        xlabel("Iteration")
        ylabel("X (m)")
        legend(strcat("Final error = ", num2str(cart_error(length(cart_error),i)), " m"));
    elseif i == 2
        xlabel("Iteration")
        ylabel("Y (m)")
        legend(strcat("Final error = ", num2str(cart_error(length(cart_error),i)), " m"))
    else
        xlabel("Iteration")
        ylabel("Z (m)")
        legend(strcat("Final error = ", num2str(cart_error(length(cart_error),i)), " m"))
    end

end