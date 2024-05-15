%https://la.mathworks.com/help/robotics/ref/importrobot.html
robot = importrobot('manipulator_description/urdf/manipulator.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

joint_goals   = table2array(readtable('matlab/data/joint_goals_square_t35_h30_p12.txt'));
joint_states  = table2array(readtable('matlab/data/joint_real_states_square_t35_h30_p12.txt'));

diferencia = joint_states - joint_goals;

for j = 1:6

    figure(1)
    subplot(3,2,j)
    plot(diferencia(:,j),"LineWidth",2);
    grid on
    grid minor
    title("Diferencia JointStates y JointGoals ");
    xlabel('Iteración');
    ylabel('Diferencia (grados)');
    legend('Diferencia') 

    figure(2);
    subplot(3,2,j)
    plot(joint_states(:,j),"LineWidth",2)
    grid on
    grid minor
    title("Desplazamiento del Joint " + (j - 1))
    xlabel("Iteración")
    ylabel("Desplazamiento (grados)")
    hold on
    plot(joint_goals(:,j),"LineWidth",2)
    legend('joint_states','joint_goals') 
    
end


