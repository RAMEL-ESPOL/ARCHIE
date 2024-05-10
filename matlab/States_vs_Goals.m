%https://la.mathworks.com/help/robotics/ref/importrobot.html
robot = importrobot('manipulator_description/urdf/manipulator.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

joint_goals_h20   = table2array(readtable('matlab/data/joint_goals_square_t5_h20_p22.txt'));
joint_states_h20  = table2array(readtable('matlab/data/joint_real_states_square_t5_h20_p22.txt'));
joint_goals_h25   = table2array(readtable('matlab/data/joint_goals_square_t5_h25_p22.txt'));
joint_states_h25  = table2array(readtable('matlab/data/joint_real_states_square_t5_h25_p22.txt'));
joint_goals_h30   = table2array(readtable('matlab/data/joint_goals_square_t5_h30_p22.txt'));
joint_states_h30  = table2array(readtable('matlab/data/joint_real_states_square_t5_h30_p22.txt'));
joint_goals_h35   = table2array(readtable('matlab/data/joint_goals_square_t5_h35_p22.txt'));
joint_states_h35  = table2array(readtable('matlab/data/joint_real_states_square_t5_h35_p22.txt'));

for i=1:length(joint_goals_h20)
    det_states_h20(i)  = det(geometricJacobian(robot,deg2rad(joint_states_h20(i,:)),'link_6'));
    g_states_h20 (i,:) = gravityTorque(robot,deg2rad(joint_states_h20(i,:)));
end
for i=1:length(joint_goals_h25)
    det_states_h25(i)  = det(geometricJacobian(robot,deg2rad(joint_states_h25(i,:)),'link_6'));
    g_states_h25 (i,:) = gravityTorque(robot,deg2rad(joint_states_h25(i,:)));
end
for i=1:length(joint_goals_h30)
    det_states_h30(i)  = det(geometricJacobian(robot,deg2rad(joint_states_h30(i,:)),'link_6'));
    g_states_h30 (i,:) = gravityTorque(robot,deg2rad(joint_states_h30(i,:)));
end
for i=1:length(joint_states_h35)
    det_states_h35(i)  = det(geometricJacobian(robot,deg2rad(joint_states_h35(i,:)),'link_6'));
    g_states_h35 (i,:) = gravityTorque(robot,deg2rad(joint_states_h35(i,:)));
end

diferencia_h20 = joint_states_h20 - joint_goals_h20;
diferencia_h25 = joint_states_h25 - joint_goals_h25;
diferencia_h30 = joint_states_h30 - joint_goals_h30;
diferencia_h35 = joint_states_h35 - joint_goals_h35;
for j = 1:4
    figure(1);
    subplot(2,2,j)
    plot(joint_states_h20(:,j),"LineWidth",2)
    grid on
    grid minor
    title("Desplazamiento del Joint " + (j - 1))
    xlabel("Iteración")
    ylabel("Desplazamiento (grados)")
    hold on
    plot(joint_states_h25(:,j),"LineWidth",2)
    hold on
    plot(joint_states_h30(:,j),"LineWidth",2)
    hold on
    plot(joint_states_h35(:,j),"LineWidth",2)
    legend('h = 20', 'h = 25', 'h = 30', 'h = 35')
    
    figure(2);
    subplot(2,2,j)
    plot(g_states_h20(:,j),"LineWidth",2)
    grid on
    grid minor
    title("Torque en el Joint " + (j - 1))
    xlabel("Iteración")
    ylabel("Torque (kg*m)")
    hold on
    plot(g_states_h25(:,j),"LineWidth",2)
    hold on
    plot(g_states_h30(:,j),"LineWidth",2)
    hold on
    plot(g_states_h35(:,j),"LineWidth",2)
    legend('h = 20', 'h = 25', 'h = 30', 'h = 35')

    figure(3)
    subplot(2,2,j)
    plot(diferencia_h20(:,j),"LineWidth",2);
    grid on
    grid minor
    title("Diferencia JointStates y JointGoals "+ (j - 1));
    xlabel('Iteración');
    ylabel('Diferencia (grados)');
    hold on
    plot(diferencia_h25(:,j),"LineWidth",2);
    hold on
    plot(diferencia_h30(:,j),"LineWidth",2);
    hold on
    plot(diferencia_h35(:,j),"LineWidth",2);
    legend('h = 20', 'h = 25', 'h = 30', 'h = 35')
end

figure(4);
plot(det_states_h20,"LineWidth",2)
grid on
grid minor
title(strcat("Determinantes de figura"))
xlabel("Iteración")
ylabel("Determinante")
hold on;
plot(det_states_h25,"LineWidth",2)
hold on;
plot(det_states_h30,"LineWidth",2)
hold on;
plot(det_states_h35,"LineWidth",2)
hold on
plot([1, length(det_states_h35)], [0, 0], 'r--');
legend('h = 20', 'h = 25', 'h = 30', 'h = 35', 'Eje 0')




