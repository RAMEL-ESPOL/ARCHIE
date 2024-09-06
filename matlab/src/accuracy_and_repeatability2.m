robot = importrobot('archie_description\urdf\manipulator2.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

experiments = 7;
samples = 10;
cart_states = [];
target_points = [];

for i=1:samples
    % Construir el nombre del archivo
    filename_real = strcat('matlab/data/', num2str(i), '_joint_real_states_precision_x0_y20_z20.txt');
    
    % Leer estados reales
    joint_states = deg2rad(table2array(readtable(filename_real)));
    tform_states = getTransform(robot, joint_states(end, : ), 'link_6'); 
    pos_states = tform2trvec(tform_states)*1000;
    cart_states = [cart_states; pos_states];
    
    x_target = 0  *1000;
    y_target = 0.2*1000;
    z_target = 0.2*1000;
    
    target_points = [target_points; x_target, y_target, z_target];
end

for i=1:samples
    % Construir el nombre del archivo
    filename_real = strcat('matlab/data/', num2str(i), '_joint_real_states_precision_x15_y15_z20.txt');
    
    % Leer estados reales
    joint_states = deg2rad(table2array(readtable(filename_real)));
    tform_states = getTransform(robot, joint_states(end, : ), 'link_6'); 
    pos_states = tform2trvec(tform_states)*1000;
    cart_states = [cart_states; pos_states];
    
    x_target = 0.15*1000;
    y_target = 0.15*1000;
    z_target = 0.20*1000;
    
    target_points = [target_points; x_target, y_target, z_target];
end

for i=1:samples
    % Construir el nombre del archivo
    filename_real = strcat('matlab/data/', num2str(i), '_joint_real_states_precision_x_15_y15_z20.txt');
    
    % Leer estados reales
    joint_states = deg2rad(table2array(readtable(filename_real)));
    tform_states = getTransform(robot, joint_states(end, : ), 'link_6'); 
    pos_states = tform2trvec(tform_states)*1000;
    cart_states = [cart_states; pos_states];
    
    x_target =-0.15*1000;
    y_target = 0.15*1000;
    z_target = 0.20*1000;
    
    target_points = [target_points; x_target, y_target, z_target];
end

for i=1:samples
    % Construir el nombre del archivo
    filename_real = strcat('matlab/data/', num2str(i), '_joint_real_states_precision_x10_y18_z20.txt');
    
    % Leer estados reales
    joint_states = deg2rad(table2array(readtable(filename_real)));
    tform_states = getTransform(robot, joint_states(end, : ), 'link_6'); 
    pos_states = tform2trvec(tform_states)*1000;
    cart_states = [cart_states; pos_states];
    
    x_target = 0.1*1000;
    y_target = 0.18*1000;
    z_target = 0.20*1000;
    
    target_points = [target_points; x_target, y_target, z_target];
end

for i=1:samples
    % Construir el nombre del archivo
    filename_real = strcat('matlab/data/', num2str(i), '_joint_real_states_precision_x_10_y18_z20.txt');
    
    % Leer estados reales
    joint_states = deg2rad(table2array(readtable(filename_real)));
    tform_states = getTransform(robot, joint_states(end, : ), 'link_6'); 
    pos_states = tform2trvec(tform_states)*1000;
    cart_states = [cart_states; pos_states];
    
    x_target = -0.1*1000;
    y_target = 0.18*1000;
    z_target = 0.20*1000;
    
    target_points = [target_points; x_target, y_target, z_target];
end

for i=1:samples
    % Construir el nombre del archivo
    filename_real = strcat('matlab/data/', num2str(i), '_joint_real_states_precision_x0_y18_z20.txt');
    
    % Leer estados reales
    joint_states = deg2rad(table2array(readtable(filename_real)));
    tform_states = getTransform(robot, joint_states(end, : ), 'link_6'); 
    pos_states = tform2trvec(tform_states)*1000;
    cart_states = [cart_states; pos_states];
    
    x_target = 0*1000;
    y_target = 0.18*1000;
    z_target = 0.20*1000;
    
    target_points = [target_points; x_target, y_target, z_target];
end

for i=1:samples
    % Construir el nombre del archivo
    filename_real = strcat('matlab/data/', num2str(i), '_joint_real_states_precision_x0_y15_z20.txt');
    
    % Leer estados reales
    joint_states = deg2rad(table2array(readtable(filename_real)));
    tform_states = getTransform(robot, joint_states(end, : ), 'link_6'); 
    pos_states = tform2trvec(tform_states)*1000;
    cart_states = [cart_states; pos_states];
    
    x_target = 0*1000;
    y_target = 0.15*1000;
    z_target = 0.20*1000;
    
    target_points = [target_points; x_target, y_target, z_target];
end

% Graficar resultados
figure;
hold on;
colors = lines(samples*experiments);

for i=1:samples*experiments
    plot(cart_states(i,1), cart_states(i,2), 'o', 'MarkerSize', 3, 'MarkerEdgeColor', colors(i,:), 'MarkerFaceColor', colors(i,:));
    plot(target_points(i,1), target_points(i,2), 'x', 'MarkerSize', 10, 'Color', colors(1,:));
end

% Etiquetas y leyenda
xlabel('X (mm)');
ylabel('Y (mm)');
title('Pruebas de Exactitud y Repetibilidad');
legend('Punto Alcanzado', 'Punto Objetivo', 'Location', 'Best');
xlim([-160 160]);
ylim([148 203]);
grid on;
grid minor;
hold off;

% Calcular el error de exactitud
error_abs = sqrt(sum((cart_states - target_points).^2, 2)); % Error absoluto en milímetros

% Graficar error en un gráfico de barras
figure;
bar(error_abs, 'FaceColor', [0.2 0.2 0.5]);
xlabel('Número de prueba');
ylabel('Error Absoluto (mm)');
title('Error de Exactitud en las Pruebas');
grid on;



