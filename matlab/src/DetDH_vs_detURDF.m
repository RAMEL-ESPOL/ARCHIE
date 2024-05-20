%https://la.mathworks.com/help/robotics/ref/importrobot.html
robot = importrobot('manipulator_description/urdf/manipulator.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];


joint_data = table2array(readtable(strcat('matlab/data/joint_goals_square_t5_h20_p15.txt')));

for i = 1:length(joint_data)
    det_urdf(i) = det(geometricJacobian(robot,deg2rad(joint_data(i,:)),'link_6'));
    det_DH  (i) = double(subs(det_J, jointVars, joint_data(i,:)));
end

figure(1);

plot(det_urdf,"LineWidth",2)
grid on
grid minor
title("Determinante del cuadrado")
xlabel("Iteración")
ylabel("Determinante")
hold on;

plot(det_DH,"LineWidth",2)
hold on;


legend('Determinante usando URDF', 'Determinante usando los parámetros DH')
plot([1, length(det_square)], [0, 0], 'r--');
