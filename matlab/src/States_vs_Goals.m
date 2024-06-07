%https://la.mathworks.com/help/robotics/ref/importrobot.html
robot = importrobot('manipulator_description/urdf/manipulator.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

joint_goals_h40   = table2array(readtable('matlab/data/joint_goals_square_t35_h40_p12.txt'));
joint_states_h40  = table2array(readtable('matlab/data/joint_real_states_square_t35_h40_p12.txt'));
% joint_goals_h45   = table2array(readtable('matlab/data/joint_goals_square_t35_h45_p12.txt'));
% joint_states_h45  = table2array(readtable('matlab/data/joint_real_states_square_t35_h45_p12.txt'));
% joint_goals_h20   = table2array(readtable('matlab/data/joint_goals_square_t5_h20_p22.txt'));
% joint_states_h20  = table2array(readtable('matlab/data/joint_real_states_square_t5_h20_p22.txt'));
% joint_goals_h25   = table2array(readtable('matlab/data/joint_goals_square_t5_h25_p22.txt'));
% joint_states_h25  = table2array(readtable('matlab/data/joint_real_states_square_t5_h25_p22.txt'));
joint_goals_h30   = table2array(readtable('matlab/data/joint_goals_square_t5_h30_p22.txt'));
joint_states_h30  = table2array(readtable('matlab/data/joint_real_states_square_t5_h30_p22.txt'));
% joint_goals_h35   = table2array(readtable('matlab/data/joint_goals_square_t5_h35_p22.txt'));
% joint_states_h35  = table2array(readtable('matlab/data/joint_real_states_square_t5_h35_p22.txt'));

for i=1:length(joint_goals_h30)
    det_states_h30(i)  = det(geometricJacobian(robot,deg2rad(joint_states_h30(i,:)),'link_6'));
    g_states_h30 (i,:) = gravityTorque(robot,deg2rad(joint_states_h30(i,:)));
end
% for i=1:length(joint_goals_h35)
%     det_states_h35(i)  = det(geometricJacobian(robot,deg2rad(joint_states_h35(i,:)),'link_6'));
%     g_states_h35 (i,:) = gravityTorque(robot,deg2rad(joint_states_h35(i,:)));
% end
for i=1:length(joint_goals_h40)
    det_states_h40(i)  = det(geometricJacobian(robot,deg2rad(joint_states_h40(i,:)),'link_6'));
    g_states_h40 (i,:) = gravityTorque(robot,deg2rad(joint_states_h40(i,:)));
end
% for i=1:length(joint_states_h45)
%     det_states_h45(i)  = det(geometricJacobian(robot,deg2rad(joint_states_h45(i,:)),'link_6'));
%     g_states_h45 (i,:) = gravityTorque(robot,deg2rad(joint_states_h45(i,:)));
% end

diferencia_h30 = abs(joint_states_h30 - joint_goals_h30);
% diferencia_h35 = joint_states_h35 - joint_goals_h35;
diferencia_h40 = abs(joint_states_h40 - joint_goals_h40);
% diferencia_h45 = joint_states_h45 - joint_goals_h45;
% 
for j = 1:4
    figure(1);
    subplot(2,2,j)
    plot(joint_states_h30(:,j),"LineWidth",2)
    grid on
    grid minor
    title("Position of  Joint " + (j))
    xlabel("Iteration")
    ylabel("Position (degrees)")
    hold on
    plot(joint_goals_h30(:,j),"LineWidth",2)
    hold on
    plot(joint_states_h40(:,j),"LineWidth",2)
    hold on
    plot(joint_goals_h40(:,j),"LineWidth",2)
    legend('Joint states, base distance = 30', 'Joint goals, base distance = 30', 'Joint states, base distance = 40', 'Joint goals, base distance = 40','Axis Y = 0') 
    axis([0 length(joint_goals_h40) -10 20])
    
    figure(2);
    subplot(2,2,j)
    plot(g_states_h30(:,j),"LineWidth",2)
    grid on
    grid minor
    title("Torque of Joint " + (j))
    xlabel("Iteration")
    ylabel("Torque (kg*m)")
    hold on
%     plot(g_states_h35(:,j),"LineWidth",2)
%     hold on
    plot(g_states_h40(:,j),"LineWidth",2)
%     hold on
%     plot(g_states_h45(:,j),"LineWidth",2)
    legend('h = 30', 'h = 35', 'Axis Y = 0') 
    
    figure(3)
    subplot(2,2,j)
    plot(diferencia_h30(:,j),"LineWidth",2);
    grid on
    grid minor
    title("Error between states and goals of joint "+ (j) + ", average = " + mean(diferencia_h30(:,j)) + " degrees");
    xlabel('Iteration');
    ylabel('Error (degrees)');
    hold on
%     plot(diferencia_h35(:,j),"LineWidth",2);
%     hold on
    plot(diferencia_h40(:,j),"LineWidth",2);
    legend('Distance from base center = 30', 'Distance from base center = 40') 
    axis([0 length(diferencia_h40) -0.5 1.5])
end

% figure(4);
% plot(det_states_h30,"LineWidth",2)
% grid on
% grid minor
% title(strcat("Determinantes de figura"))
% xlabel("Iteration")
% ylabel("Determinante")
% hold on;
% plot(det_states_h35,"LineWidth",2)
% hold on;
% plot(det_states_h40,"LineWidth",2)
% hold on;
% plot(det_states_h45,"LineWidth",2)
% hold on
% plot([1, length(det_states_h35)], [0, 0], 'r--');
% legend('h = 30', 'h = 35', 'h = 40', 'h = 45', 'Axis Y = 0')


% for j = 1:6
%     figure(5);
%     subplot(3,2,j)
%     plot(joint_states_h35(:,j),"LineWidth",2)
%     grid on
%     grid minor
%     title("Position of  Joint " + (j - 1))
%     xlabel("Iteration")
%     ylabel("Position (degrees)")
%     hold on
%     plot(joint_goals_h35(:,j),"LineWidth",2)
%     legend('joint_states','joint_goals') 
%     %axis([0 400 -90 90])
%     
%     figure(3)
%     subplot(3,2,j)
%     plot(diferencia_h35(:,j),"LineWidth",2);
%     grid on
%     grid minor
%     title("Diferencia JointStates y JointGoals "+ (j - 1));
%     xlabel('Iteration');
%     ylabel('Diferencia (grados)');
%     hold on
% end

% figure(6)
% plot(joint_states_h35(:,2))
% grid on
% grid minor
% title("Position of  Joint " + 1)
% xlabel("Iteration")
% ylabel("Position (degrees)")
% hold on
% plot(joint_goals_h35(:,2))
% %axis([0 400 -180 180])
% legend('joint_states','joint_goals') 
% 



