robot = importrobot('archie_description\urdf\manipulator2.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

ndoc = 11;
% 
% joint_goals   = deg2rad(table2array(readtable('matlab/data/joint_goals_square_t5_h30_p22.txt')));
% joint_states  = deg2rad(table2array(readtable('matlab/data/joint_real_states_square_t5_h30_p22.txt')));

% joint_goals   = deg2rad(table2array(readtable('matlab/data/joint_goals_square_t5_h_p15.txt')));
% joint_states  = deg2rad(table2array(readtable('matlab/data/joint_real_states_square_t5_h20_p15.txt')));
    
joint_goals   = deg2rad(table2array(readtable('matlab/data/joint_goals_circle_t0_h30_p22.txt')));
joint_states  = deg2rad(table2array(readtable('matlab/data/joint_real_states_circle_t0_h30_p22.txt')));

% joint_goals   = deg2rad(table2array(readtable('matlab/data/joint_goals_triangle_t0_h30_p22.txt')));
% joint_states  = deg2rad(table2array(readtable('matlab/data/joint_real_states_triangle_t0_h30_p22.txt')));

cart_states = [];
cart_goals = [];

for i=1:length(joint_goals)
    tform_states = getTransform(robot, joint_states(i, : ), 'link_6'); 
    pos_states = tform2trvec(tform_states)*1000;
    cart_states = [cart_states; pos_states];
    
    tform_goals = getTransform(robot, joint_goals(i, : ), 'link_6'); 
    pos_goals = tform2trvec(tform_goals)*1000;
    cart_goals = [cart_goals; pos_goals];
end

% Aplicar condición para Z y eliminar filas donde Z >= 0.27
cond_goals = cart_goals(:, 3) < 222; % Puntos en los que escribe
cart_goals = cart_goals(cond_goals, :); % Filtrar filas completas

cond_states = cart_states(:, 3) < 222; % Puntos en los que escribe
cart_states = cart_states(cond_states, :); % Filtrar filas completas

colors = lines(3); % Define a set of colors

figure(2);
plot(cart_states(:,1), cart_states(:,2),"LineWidth",2, 'Color', colors(1,:))
xlim([-85, 85])
ylim([215, 305])
ylabel('Y (mm)', 'FontSize', 12);
xlabel('X (mm)', 'FontSize', 12);

hold on
plot(cart_goals(:,1), cart_goals(:,2),"LineWidth",2, 'Color', colors(2,:))
grid on
grid minor
legend("Final Figure", "Expected Figure","LineWidth",2, 'FontSize', 20)
