%https://la.mathworks.com/help/robotics/ref/importrobot.html
robot = importrobot('manipulator_description/urdf/manipulator.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

robot2 = importrobot('manipulator_final2\urdf\manipulator_final2.urdf');
robot2.DataFormat = 'row';
robot2.Gravity = [0 0 -9.81];

joint_data = table2array(readtable(strcat('matlab/data/joint_goals_square_t5_h20_p15.txt')));

for i = 1:length(joint_data)
    J_urdf = geometricJacobian(robot,deg2rad(joint_data(i,:)),'link_6');
    J_urdf2 = geometricJacobian(robot2,deg2rad(joint_data(i,:)),'link_6');
    det_urdf(i) = det(J_urdf);
    det_urdf2(i) = det(J_urdf2);
    det_DH   (i) = double(subs(det_J, jointVars, joint_data(i,:)));
end

figure(1);

plot(det_urdf,"LineWidth",2)
grid on
grid minor
title("Determinante del cuadrado")
xlabel("Iteración")
ylabel("Determinante")
hold on;

plot(det_urdf2,"LineWidth",2)
hold on;

plot(det_DH,"LineWidth",2)
hold on;

legend('Determinante usando antiguo URDF','Determinante usando nuevo URDF', 'Determinante usando los parámetros DH')
plot([1, length(det_DH)], [0, 0], 'r--');
