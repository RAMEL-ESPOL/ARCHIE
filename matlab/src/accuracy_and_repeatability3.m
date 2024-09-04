robot = importrobot('archie_description\urdf\manipulator2.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

samples = 10;
errors = zeros(samples*3, 2); % Para almacenar errores en X y Y

for j = 1:3
    for i=1:samples
        % Construir el nombre del archivo
        if j == 1
            filename_real = strcat('matlab/data/', num2str(i), '_joint_real_states_precision_x0_y20_z20.txt');
            x_target = 0;
            y_target = 0.2;
        elseif j == 2
            filename_real = strcat('matlab/data/', num2str(i), '_joint_real_states_precision_x15_y15_z20.txt');
            x_target = 0.15;
            y_target = 0.15;
        else
            filename_real = strcat('matlab/data/', num2str(i), '_joint_real_states_precision_x_15_y15_z20.txt');
            x_target = -0.15;
            y_target = 0.15;
        end

        % Leer estados reales
        joint_states = deg2rad(table2array(readtable(filename_real)));
        tform_states = getTransform(robot, joint_states(end, : ), 'link_6'); 
        pos_states = tform2trvec(tform_states);

        % Calcular error
        error_x = abs(pos_states(1) - x_target);
        error_y = abs(pos_states(2) - y_target);

        errors((j-1)*samples + i, :) = [error_x, error_y];
    end
end

% Graficar errores como barras
figure;
subplot(2,1,1);
bar(errors(:,1)*1000, 'FaceColor', [0 0.5 0.5]);
title('Error en la coordenada X (mm)');
xlabel('Muestra');
ylabel('Error (mm)');

subplot(2,1,2);
bar(errors(:,2)*1000, 'FaceColor', [0.5 0 0.5]);
title('Error en la coordenada Y (mm)');
xlabel('Muestra');
ylabel('Error (mm)');

sgtitle('Análisis de Errores de Exactitud y Repetibilidad');
grid on;