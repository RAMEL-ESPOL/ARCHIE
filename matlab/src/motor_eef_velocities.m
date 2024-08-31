robot = importrobot('archie_description\urdf\manipulator.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

ndoc = 6;

motor_pos = importdata(strcat('matlab/data_pwm/',num2str(ndoc),'_motor_position.txt'));
motor_vel = importdata(strcat('matlab/data_pwm/',num2str(ndoc),'_motor_velocity.txt'));
motor_vel = motor_vel*2*pi/60; % conversion from rpm to rad/s
velocity_eef = [];

%https://la.mathworks.com/help/robotics/ref/rigidbodytree.geometricjacobian.html
for i=1:length(motor_vel)
    jacob = geometricJacobian(robot, motor_pos(i, : ), 'link_6');        
    linear_vel = (jacob*(motor_vel(i,:).')); % J(q)(6x6) * q' (6x1)
    velocity_eef = [velocity_eef; (linear_vel.')]; % We use the linear_vel transpose to get a row vector
end

colors = lines(6); % Define a set of colors

figure(1);
sgtitle('End-Effector Velocities', 'FontSize', 15); % Super title for the entire figure

subplot(3,2,1)
plot(velocity_eef(:,1),"LineWidth",2, 'Color', colors(1,:))
grid on
grid minor
ylabel("Angular Vel X (rad/s)")

subplot(3,2,3)
plot(velocity_eef(:,2),"LineWidth",2, 'Color', colors(2,:))
grid on
grid minor
ylabel("Angular Vel Y (rad/s)")

subplot(3,2,5)
plot(velocity_eef(:,3),"LineWidth",2, 'Color', colors(3,:))
grid on
grid minor
xlabel("Iteration")
ylabel("Angular Vel Z (rad/s)")

subplot(3,2,2)
plot(velocity_eef(:,4),"LineWidth",2, 'Color', colors(4,:))
grid on
grid minor
ylabel("Linear Vel X (m/s)")

subplot(3,2,4)
plot(velocity_eef(:,5),"LineWidth",2, 'Color', colors(5,:))
grid on
grid minor
ylabel("Linear Vel Y (m/s)")

subplot(3,2,6)
plot(velocity_eef(:,6),"LineWidth",2, 'Color', colors(6,:))

grid on
grid minor
xlabel("Iteration")
ylabel("Linear Vel Z (m/s)")

