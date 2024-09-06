robot = importrobot('archie_description\urdf\manipulator2.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

experiments = 7;
samples = 10;
errors = zeros(samples*experiments, 3); % Para almacenar errores en X, Y y Z

for j = 1:experiments
    for i=1:samples
        % Construir el nombre del archivo
        if j == 1
            filename_real = strcat('matlab/data/', num2str(i), '_joint_real_states_precision_x0_y20_z20.txt');
            x_target = 0;
            y_target = 0.2;
            z_target = 0.2;
        elseif j == 2
            filename_real = strcat('matlab/data/', num2str(i), '_joint_real_states_precision_x15_y15_z20.txt');
            x_target = 0.15;
            y_target = 0.15;
            z_target = 0.2;
        elseif j == 3
            filename_real = strcat('matlab/data/', num2str(i), '_joint_real_states_precision_x_15_y15_z20.txt');
            x_target = -0.15;
            y_target = 0.15;
            z_target = 0.2;
        elseif j == 4
            filename_real = strcat('matlab/data/', num2str(i), '_joint_real_states_precision_x10_y18_z20.txt');
            x_target = 0.1;
            y_target = 0.18;
            z_target = 0.2;
        elseif j == 5
            filename_real = strcat('matlab/data/', num2str(i), '_joint_real_states_precision_x_10_y18_z20.txt');
            x_target = -0.1;
            y_target = 0.18;
            z_target = 0.2;
        elseif j == 6
            filename_real = strcat('matlab/data/', num2str(i), '_joint_real_states_precision_x0_y18_z20.txt');
            x_target = 0;
            y_target = 0.18;
            z_target = 0.2;
        elseif j == 7
            filename_real = strcat('matlab/data/', num2str(i), '_joint_real_states_precision_x0_y15_z20.txt');
            x_target = 0;
            y_target = 0.15;
            z_target = 0.2;
        end

        % Leer estados reales
        joint_states = deg2rad(table2array(readtable(filename_real)));
        tform_states = getTransform(robot, joint_states(end, : ), 'link_6'); 
        pos_states = tform2trvec(tform_states);

        % Calcular error
        error_x = pos_states(1) - x_target;
        error_y = pos_states(2) - y_target;
        error_z = pos_states(3) - z_target;

        errors((j-1)*samples + i, :) = [error_x, error_y, error_z];
    end
end

% Calcular magnitud del error total
error_magnitude = sqrt(sum(errors.^2, 2)); % Error total en X, Y y Z

% Graficar KDE del error
figure;
hold on;

% Histograma suavizado (KDE)
[f, xi] = ksdensity(error_magnitude*1000); % Convertir a mm
plot(xi, f, 'LineWidth', 2);

% Etiquetas y leyenda
xlabel('Magnitud del Error (mm)');
ylabel('Densidad de Probabilidad');
title('Densidad de Probabilidad del Error en X, Y y Z');
grid on;
hold off;
