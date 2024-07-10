%https://la.mathworks.com/help/robotics/ref/importrobot.html
robot = importrobot('archie_description\urdf\manipulator.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];


joint_data = table2array(readtable(strcat('matlab/data/joint_goals_square_t5_h30_p22.txt')));

for i = 1:length(joint_data)
    J_urdf = geometricJacobian(robot,deg2rad(joint_data(i,:)),'link_6');
    J3 = J_urdf(1:3, :);
    J_urdf(1:3, :) = J_urdf(4:6, :);
    J_urdf(4:6,:) = J3;
    det_urdf(i) = det(J_urdf);
    det_DH   (i) = double(subs(det_J, jointVars, joint_data(i,:)));
end

figure(1);

plot(det_DH,"LineWidth",2)
grid on
grid minor
title("Determinante URDF vs Determinante DH Parameters")
xlabel("Iteración")
ylabel("Determinante")
hold on;

plot(det_urdf,"LineWidth",2)
hold on;

iteration= 0:1:length(det_DH);

% Encuentra los índices donde el determinante es cero
zero_indices = find(det_DH < 0.0001 & det_DH > -0.0001);

% Marca los puntos donde el determinante es cero
scatter(iteration(zero_indices), det_DH(zero_indices), 'r', 'filled');

legend('Determinante usando los parámetros DH', 'Determinante usando URDF', 'Puntos en que el determinante es 0')
axis([0 length(det_DH) -0.025 0.025])
