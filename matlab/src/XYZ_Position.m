%https://la.mathworks.com/help/robotics/ref/importrobot.html
robot = importrobot('manipulator_description/urdf/manipulator_without_pen.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

joint_goals_h40   = deg2rad(table2array(readtable('matlab/data/joint_goals_square_t35_h40_p12.txt')));

joint_goals_h30   = deg2rad(table2array(readtable('matlab/data/joint_goals_square_t5_h30_p22.txt')));

positions_40 = [];
positions_30 = [];
for i=1:length(joint_goals_h40)
    tform = getTransform(robot, joint_goals_h40(i, : ), 'link_6'); 
    pos = tform2trvec(tform);
    positions_40 = [positions_40; pos];
end

for i=1:length(joint_goals_h30)
    tform = getTransform(robot, joint_goals_h30(i, : ), 'link_6'); 
    pos = tform2trvec(tform);
    positions_30 = [positions_30; pos];
end

for i=1:3
    figure(1);
    subplot(3,1,i)
    plot(positions_40(:,i),"LineWidth",2)
    grid on
    grid minor
    if i == 1
        title("Position of the end-effector")
        xlabel("Iteration")
        ylabel("X (m)")
    elseif i == 2
        xlabel("Iteration")
        ylabel("Y (m)")
    else
        xlabel("Iteration")
        ylabel("Z (m)")
    end
% 
%     hold on
%     plot(positions_30(:,i),"LineWidth",2)    
%     legend('Y distance = 0.4', 'Y distance = 0.3')
%     
    
    
    
    
    
    
    
end