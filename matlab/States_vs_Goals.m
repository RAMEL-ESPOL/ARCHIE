%https://la.mathworks.com/help/robotics/ref/importrobot.html
robot = importrobot('manipulator_description/urdf/manipulator.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

joint_goals_h30   = table2array(readtable('matlab/data/joint_goals_square_t35_h30_p12.txt'));
joint_states_h30  = table2array(readtable('matlab/data/joint_real_states_square_t35_h30_p12.txt'));
joint_goals_h35   = table2array(readtable('matlab/data/joint_goals_square_t35_h35_p12.txt'));
joint_states_h35  = table2array(readtable('matlab/data/joint_real_states_square_t35_h35_p12.txt'));
joint_goals_h40   = table2array(readtable('matlab/data/joint_goals_square_t35_h40_p12.txt'));
joint_states_h40  = table2array(readtable('matlab/data/joint_real_states_square_t35_h40_p12.txt'));
joint_goals_h45   = table2array(readtable('matlab/data/joint_goals_square_t35_h45_p12.txt'));
joint_states_h45  = table2array(readtable('matlab/data/joint_real_states_square_t35_h45_p12.txt'));

for i=1:length(joint_goals_h30)
    det_states_h30(i)  = det(geometricJacobian(robot,deg2rad(joint_states_h30(i,:)),'link_6'));
    g_states_h30 (i,:) = gravityTorque(robot,deg2rad(joint_states_h30(i,:)));
end
for i=1:length(joint_goals_h35)
    det_states_h35(i)  = det(geometricJacobian(robot,deg2rad(joint_states_h35(i,:)),'link_6'));
    g_states_h35 (i,:) = gravityTorque(robot,deg2rad(joint_states_h35(i,:)));
end
for i=1:length(joint_goals_h40)
    det_states_h40(i)  = det(geometricJacobian(robot,deg2rad(joint_states_h40(i,:)),'link_6'));
    g_states_h40 (i,:) = gravityTorque(robot,deg2rad(joint_states_h40(i,:)));
end
for i=1:length(joint_states_h45)
    det_states_h45(i)  = det(geometricJacobian(robot,deg2rad(joint_states_h45(i,:)),'link_6'));
    g_states_h45 (i,:) = gravityTorque(robot,deg2rad(joint_states_h45(i,:)));
end

diferencia_h30 = joint_states_h30 - joint_goals_h30;
diferencia_h35 = joint_states_h35 - joint_goals_h35;
diferencia_h40 = joint_states_h40 - joint_goals_h40;
diferencia_h45 = joint_states_h45 - joint_goals_h45;
% 
% for j = 1:4
%     figure(1);
%     subplot(2,2,j)
%     plot(joint_states_h30(:,j),"LineWidth",2)
%     grid on
%     grid minor
%     title("Desplazamiento del Joint " + (j - 1))
%     xlabel("Iteración")
%     ylabel("Desplazamiento (grados)")
%     hold on
%     plot(joint_states_h35(:,j),"LineWidth",2)
%     hold on
%     plot(joint_states_h40(:,j),"LineWidth",2)
%     hold on
%     plot(joint_states_h45(:,j),"LineWidth",2)
%     legend('h = 30', 'h = 35', 'h = 40', 'h = 45', 'Eje 0') 
%     
%     figure(2);
%     subplot(2,2,j)
%     plot(g_states_h30(:,j),"LineWidth",2)
%     grid on
%     grid minor
%     title("Torque en el Joint " + (j - 1))
%     xlabel("Iteración")
%     ylabel("Torque (kg*m)")
%     hold on
%     plot(g_states_h35(:,j),"LineWidth",2)
%     hold on
%     plot(g_states_h40(:,j),"LineWidth",2)
%     hold on
%     plot(g_states_h45(:,j),"LineWidth",2)
%     legend('h = 30', 'h = 35', 'h = 40', 'h = 45', 'Eje 0') 
% 
%     figure(3)
%     subplot(2,2,j)
%     plot(diferencia_h30(:,j),"LineWidth",2);
%     grid on
%     grid minor
%     title("Diferencia JointStates y JointGoals "+ (j - 1));
%     xlabel('Iteración');
%     ylabel('Diferencia (grados)');
%     hold on
%     plot(diferencia_h35(:,j),"LineWidth",2);
%     hold on
%     plot(diferencia_h40(:,j),"LineWidth",2);
%     hold on
%     plot(diferencia_h45(:,j),"LineWidth",2);
%     legend('h = 30', 'h = 35', 'h = 40', 'h = 45', 'Eje 0') 
% end
% 
% figure(4);
% plot(det_states_h30,"LineWidth",2)
% grid on
% grid minor
% title(strcat("Determinantes de figura"))
% xlabel("Iteración")
% ylabel("Determinante")
% hold on;
% plot(det_states_h35,"LineWidth",2)
% hold on;
% plot(det_states_h40,"LineWidth",2)
% hold on;
% plot(det_states_h45,"LineWidth",2)
% hold on
% plot([1, length(det_states_h35)], [0, 0], 'r--');
% legend('h = 30', 'h = 35', 'h = 40', 'h = 45', 'Eje 0')


for j = 1:6
    figure(5);
    subplot(3,2,j)
    plot(joint_states_h35(:,j),"LineWidth",2)
    grid on
    grid minor
    title("Desplazamiento del Joint " + (j - 1))
    xlabel("Iteración")
    ylabel("Desplazamiento (grados)")
    hold on
    plot(joint_goals_h35(:,j),"LineWidth",2)
    legend('joint_states','joint_goals') 
end

figure(6)
plot(joint_states_h35(:,4))
grid on
grid minor
title("Desplazamiento del Joint " + 3)
xlabel("Iteración")
ylabel("Desplazamiento (grados)")
hold on
plot(joint_goals_h35(:,4))
legend('joint_states','joint_goals') 




