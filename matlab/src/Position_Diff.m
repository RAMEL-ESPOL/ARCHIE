%https://la.mathworks.com/help/robotics/ref/importrobot.html
robot = importrobot('archie_description/urdf/manipulator.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

joint_goals   = table2array(readtable('matlab/data/joint_goals_square_t5_h25_p22.txt'));
joint_states  = table2array(readtable('matlab/data/joint_real_states_square_t5_h25_p22.txt'));

diferencia = abs(joint_states - joint_goals);
for i= 1 :4
    figure(i);
    subplot(2,1,1)
    plot(joint_states(:,i),"LineWidth",2)
    hold on
    plot(joint_goals(:,i),"LineWidth",2)
    grid on
    grid minor
    title("Joints positions and errors during the square path", 'FontSize', 22)
    ylabel(strcat("Position (deg)"), 'FontSize', 18)
    legend("Joint " + num2str(i) + " states", "Joint " + num2str(i) + " goals"', 'FontSize', 20)
    axis([0 length(joint_goals) -20 30])
    set(gca, 'Position', [0.13, 0.55, 0.77, 0.35]) % Ajuste posición del subplot superior
    set(gca, 'FontSize', 24) % Cambiar tamaño de fuente de los ejes

    subplot(2,1,2)
    plot(diferencia(:,i),"LineWidth",2);
    grid on
    grid minor
    xlabel('Iteration', 'FontSize', 12);
    ylabel(strcat("Error (deg)"), 'FontSize', 15);
    legend(strcat("Joint " + num2str(i) + " average error:  " ,num2str(round(mean(diferencia(:,i)),3))), 'FontSize', 20)
    axis([0 length(diferencia) -0.2 2.5])
    set(gca, 'Position', [0.13, 0.2, 0.77, 0.35]) % Ajuste posición del subplot inferior
    set(gca, 'FontSize', 24) % Cambiar tamaño de fuente de los ejes
end

