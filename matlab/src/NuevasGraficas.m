%https://la.mathworks.com/help/robotics/ref/importrobot.html
robot = importrobot('manipulator_description/urdf/manipulator.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];


joint_data_square_t0_h15_p12 = table2array(readtable(strcat('matlab/data/joint_goals_square_t0_h15_p12.txt')));
joint_data_square_t0_h15_p23 = table2array(readtable(strcat('matlab/data/joint_goals_square_t0_h15_p23.txt')));
joint_data_square_t0_h25_p12 = table2array(readtable(strcat('matlab/data/joint_goals_square_t0_h25_p12.txt')));
joint_data_square_t0_h25_p23 = table2array(readtable(strcat('matlab/data/joint_goals_square_t0_h25_p23.txt')));
joint_data_square_t0_h35_p12 = table2array(readtable(strcat('matlab/data/joint_goals_square_t0_h35_p12.txt')));
joint_data_square_t30_h25_p0 = table2array(readtable(strcat('matlab/data/joint_goals_square_t30_h25_p0.txt')));
joint_data_square_t30_h30_p0 = table2array(readtable(strcat('matlab/data/joint_goals_square_t30_h30_p0.txt')));
joint_data_square_t30_h35_p0 = table2array(readtable(strcat('matlab/data/joint_goals_square_t30_h35_p0.txt')));

for i = 1:length(joint_data_square_t0_h15_p12)
    det_squaret0_h15_p12(i) = det(geometricJacobian(robot,deg2rad(joint_data_square_t0_h15_p12(i,:)),'link_6'));
end
for i = 1:length(joint_data_square_t0_h15_p23)
    det_squaret0_h15_p23(i) = det(geometricJacobian(robot,deg2rad(joint_data_square_t0_h15_p23(i,:)),'link_6'));
end
for i = 1:length(joint_data_square_t0_h25_p12)
    det_squaret0_h25_p12(i) = det(geometricJacobian(robot,deg2rad(joint_data_square_t0_h25_p12(i,:)),'link_6'));
end
for i = 1:length(joint_data_square_t0_h25_p23)
    det_squaret0_h25_p23(i) = det(geometricJacobian(robot,deg2rad(joint_data_square_t0_h25_p23(i,:)),'link_6'));
end
for i = 1:length(joint_data_square_t0_h35_p12)
    det_squaret0_h35_p12(i) = det(geometricJacobian(robot,deg2rad(joint_data_square_t0_h35_p12(i,:)),'link_6'));
end
for i = 1:length(joint_data_square_t30_h25_p0)
    det_squaret30_h25_p0(i) = det(geometricJacobian(robot,deg2rad(joint_data_square_t30_h25_p0(i,:)),'link_6'));
end
for i = 1:length(joint_data_square_t30_h30_p0)
    det_squaret30_h30_p0(i) = det(geometricJacobian(robot,deg2rad(joint_data_square_t30_h30_p0(i,:)),'link_6'));
end
for i = 1:length(joint_data_square_t30_h35_p0)
    det_squaret30_h35_p0(i) = det(geometricJacobian(robot,deg2rad(joint_data_square_t30_h35_p0(i,:)),'link_6'));
end

figure(1);

plot(det_squaret0_h15_p12,"LineWidth",2)
grid on
grid minor
title("Determinante del cuadrado")
xlabel("Iteración")
ylabel("Determinante")
hold on;

plot(det_squaret0_h15_p23,"LineWidth",2)
hold on;

plot(det_squaret0_h25_p12,"LineWidth",2)
hold on;

plot(det_squaret0_h25_p23,"LineWidth",2)
hold on;

plot(det_squaret0_h35_p12,"LineWidth",2)
hold on;

plot(det_squaret30_h25_p0,"LineWidth",2)
hold on;

plot(det_squaret30_h30_p0,"LineWidth",2)
hold on;

plot(det_squaret30_h35_p0,"LineWidth",2)
hold on;

legend(' theta=0 y_h= 15 pen=12',' theta=0 y_h= 15 pen=23',' theta=0 y_h= 25 pen=12',' theta=0 y_h= 25 pen=23',' theta=0 y_h= 35 pen=12',' theta=30 y_h= 25 pen=0',' theta=30 y_h= 30 pen=0','theta=30 y_h= 35 pen=0')
plot([1, length(det_square)], [0, 0], 'r--');
