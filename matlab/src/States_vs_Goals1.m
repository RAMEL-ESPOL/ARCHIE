%https://la.mathworks.com/help/robotics/ref/importrobot.html
robot = importrobot('archie_description/urdf/manipulator.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

joint_goals   = table2array(readtable('matlab/data/joint_goals_espol_t5_h20_p21.txt'));
joint_states  = table2array(readtable('matlab/data/joint_real_states_espol_t5_h20_p21.txt'));

diferencia = abs(joint_states - joint_goals);

for j = 1:6

    figure(1)
    subplot(3,2,j)
    plot(diferencia(:,j),"LineWidth",2);
    grid on
    grid minor
    title("Diferencia JointStates y JointGoals " + (j - 1));
    xlabel('Iteración');
    ylabel('Diferencia (grados)');
    legend("Promedio: " + mean(diferencia(:,j)))
    axis([0 900 -2 2])

    figure(2);
    subplot(3,2,j)
    plot(joint_states(:,j),"LineWidth",2)
    grid on
    grid minor
    title("Desplazamiento del Joint " + (j - 1))
    xlabel("Iteración")
    ylabel("Desplazamiento (grados)")
    axis([0 900 -40 40])
    hold on
    plot(joint_goals(:,j),"LineWidth",2)
    legend('joint_states','joint_goals') 
    
end


