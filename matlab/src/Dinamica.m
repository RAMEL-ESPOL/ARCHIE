%https://la.mathworks.com/help/robotics/ref/importrobot.html
robot = importrobot('manipulator_description/urdf/manipulator.urdf');
robot.DataFormat = 'row';
figura = "square";
joint_data = table2array(readtable(strcat('matlab/joint_goals_', figura, '.txt')));
robot.Gravity = [0 0 -9.81];


% Jacobiano (configuracion inicial)
% https://la.mathworks.com/help/robotics/ref/rigidbodytree.geometricjacobian.html
% Valores Propios y Determinante del Jacobiano
% Dinámicas directas debidas a fuerzas externas en el modelo de árbol de cuerpo rígido
% https://la.mathworks.com/help/robotics/ref/rigidbodytree.forwarddynamics.html
% Calcular dinámicas inversas para configurar una articulación estática

for i=1:length(joint_data)
    joint_states(i,:) = joint_data(i,:);
    joint_goals(i, : ) = joint_data(i,:);
    determinante(i)=det(geometricJacobian(robot,deg2rad(joint_data(i,:)),'link_6'));
    gtau(i,:) = gravityTorque(robot,deg2rad(joint_data(i,:)));
end


diferencia = joint_states - joint_goals;
for j = 1:4
    figure(1);
    subplot(2,2,j)
    plot(joint_states(:,j),"LineWidth",2)
    grid on
    grid minor
    title("Desplazamiento Joint " + (j - 1))
    xlabel("Iteración")
    ylabel("Desplazamiento (grados)")
    hold on
    plot(joint_goals(:,j),"LineWidth",2)
    legend('Joint State', 'Joint Goal')
    
    figure(2);
    subplot(2,2,j)
    plot(gtau(:,j),"LineWidth",2)
    grid on
    grid minor
    title("Torque por Joint " + (j - 1))
    xlabel("Iteración")
    ylabel("Torque (kg*m)")

    figure(3);
    subplot(2,2,j)
    plot(diferencia(:,j),"LineWidth",2);
    grid on
    grid minor
    title("Diferencia JointStates y JointGoals "+ (j - 1));
    xlabel('Iteración');
    ylabel('Diferencia (grados)');
end



% for i=1:length(joint_states)
%     
%     M(i, :) = massMatrix(robot, joint_states(i,:))*(joint_acc(i, :))';
%     C(i,:) = -velocityProduct(robot, joint_states(i,:),joint_vel(i, :));
%     
% end
% 
% tau = M + C + gtau