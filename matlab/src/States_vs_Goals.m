%https://la.mathworks.com/help/robotics/ref/importrobot.html
robot = importrobot('archie_description/urdf/manipulator.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

% joint_goals_h40   = table2array(readtable('matlab/data/joint_goals_square_t35_h40_p12.txt'));
% joint_states_h40  = table2array(readtable('matlab/data/joint_real_states_square_t35_h40_p12.txt'));
% joint_goals_h45   = table2array(readtable('matlab/data/joint_goals_square_t35_h45_p12.txt'));
% joint_states_h45  = table2array(readtable('matlab/data/joint_real_states_square_t35_h45_p12.txt'));
% joint_goals_h20   = table2array(readtable('matlab/data/joint_goals_square_t5_h20_p22.txt'));
% joint_states_h20  = table2array(readtable('matlab/data/joint_real_states_square_t5_h20_p22.txt'));
joint_goals_h25   = table2array(readtable('matlab/data/joint_goals_square_t5_h25_p22.txt'));
joint_states_h25  = table2array(readtable('matlab/data/joint_real_states_square_t5_h25_p22.txt'));

joint_goals_h30   = table2array(readtable('matlab/data/joint_goals_square_t5_h30_p22.txt'));
joint_states_h30  = table2array(readtable('matlab/data/joint_real_states_square_t5_h30_p22.txt'));

% joint_goals_h35   = table2array(readtable('matlab/data/joint_goals_square_t5_h35_p22.txt'));
% joint_states_h35  = table2array(readtable('matlab/data/joint_real_states_square_t5_h35_p22.txt'));

% for i=1:length(joint_goals_h30)
%     det_states_h30(i)  = det(geometricJacobian(robot,deg2rad(joint_states_h30(i,:)),'link_6'));
%     g_states_h30 (i,:) = gravityTorque(robot,deg2rad(joint_states_h30(i,:)));
% end
% for i=1:length(joint_goals_h35)
%     det_states_h35(i)  = det(geometricJacobian(robot,deg2rad(joint_states_h35(i,:)),'link_6'));
%     g_states_h35 (i,:) = gravityTorque(robot,deg2rad(joint_states_h35(i,:)));
% end
% for i=1:length(joint_goals_h25)
%     det_states_h25(i)  = det(geometricJacobian(robot,deg2rad(joint_states_h25(i,:)),'link_6'));
%     g_states_h25 (i,:) = gravityTorque(robot,deg2rad(joint_states_h25(i,:)));
% end
% for i=1:length(joint_states_h45)
%     det_states_h45(i)  = det(geometricJacobian(robot,deg2rad(joint_states_h45(i,:)),'link_6'));
%     g_states_h45 (i,:) = gravityTorque(robot,deg2rad(joint_states_h45(i,:)));
% end

diferencia_h30 = abs(joint_states_h30 - joint_goals_h30);
% diferencia_h35 = joint_states_h35 - joint_goals_h35;

diferencia_h25 = abs(joint_states_h25 - joint_goals_h25);

% diferencia_h45 = joint_states_h45 - joint_goals_h45;
% 
for j = 1:4
    if j == 1 || j == 2
        figure(1);
        subplot(2,1,j)
        plot(joint_states_h30(:,j),"LineWidth",2)
        grid on
        grid minor
        if j == 1
            title("Position of joints in the square path with Y distance of 0.3")
        end
        xlabel("Iteration")
        ylabel(strcat("Joint ", num2str(j)," (degrees)"))
        hold on
        plot(joint_goals_h30(:,j),"LineWidth",2)
        legend('Joint states', 'Joint goals', 'FontSize', 15)
        axis([0 length(joint_goals_h25) -20 20])
        
        figure(2)
        subplot(2,1,j)
        plot(diferencia_h30(:,j),"LineWidth",2);
        grid on
        grid minor
        if j==1
            title("Error between joint states and joint goals ");
        end
        xlabel('Iteration');
        ylabel(strcat('Error of joint ' ,num2str(j) , '(degrees)'));
        legend(strcat('Average error:  ' ,num2str(round(mean(diferencia_h30(:,j)),3))    )  , 'FontSize', 15 )
        axis([0 length(diferencia_h30) -0.5 1.5])
        
        figure(6)
        subplot(2,1,j)
        plot(diferencia_h25(:,j),"LineWidth",2);
        grid on
        grid minor
        if j==1
            title("Error between joint states and joint goals ");
        end
        xlabel('Iteration');
        ylabel(strcat('Error of joint ' ,num2str(j) , '(degrees)'));
        legend(strcat('Average error:  ' ,num2str(round(mean(diferencia_h25(:,j)),3))), 'FontSize', 15)
        axis([0 length(diferencia_h25) -0.5 1.5])
    else
        figure(3);
        subplot(2,1,j-2)
        plot(joint_states_h30(:,j),"LineWidth",2)
        grid on
        grid minor
        if j == 3
            title("Position of joints in the square path")
        end
        xlabel("Iteration")
        ylabel(strcat("Joint " ,num2str(j) , " (degrees)"))
        hold on
        plot(joint_goals_h30(:,j),"LineWidth",2)
        axis([0 length(joint_goals_h25) -20 20])

        
        figure(4);
        subplot(2,1,j-2)
        plot(joint_states_h30(:,j),"LineWidth",2)
        grid on
        grid minor
        if j == 3
            title("Position of joints in the square path")
        end
        xlabel("Iteration")
        ylabel(strcat("Joint " ,num2str(j) , " (degrees)"))
        hold on
        plot(joint_goals_h30(:,j),"LineWidth",2)
        hold on
        legend('Joint states', 'Joint goals', 'FontSize', 15) 
        axis([0 length(joint_goals_h25) -20 20])
    end  
    
%     figure(2);
%     subplot(2,2,j)
%     plot(g_states_h30(:,j),"LineWidth",2)
%     grid on
%     grid minor
%     title("Torque of Joint " + num2str(j))
%     xlabel("Iteration")
%     ylabel("Torque (kg*m)")
%     hold on
%     plot(g_states_h35(:,j),"LineWidth",2)
%     hold on
%     plot(g_states_h25(:,j),"LineWidth",2)
%     hold on
%     plot(g_states_h45(:,j),"LineWidth",2)
%     legend('h = 30', 'h = 35', 'Axis Y = 0') 
   
    
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



