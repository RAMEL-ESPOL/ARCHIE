robot = importrobot('archie_description\urdf\manipulator2.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

experiments = 7;
samples = 10;
errors = zeros(samples*experiments, 3); % Para almacenar errores en X, Y y Z
percent_errors = zeros(samples*experiments, 3); % Para almacenar errores porcentuales

for j = 1:experiments
    for i = 1:samples
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

        % Guardar error absoluto en mm
        errors((j-1)*samples + i, :) = [error_x, error_y, error_z] * 1000;

        % Calcular error porcentual
        percent_errors((j-1)*samples + i, :) = [error_x / x_target, error_y / y_target, error_z / z_target] * 100;
    end
end

% --- Graficar la densidad de probabilidad del error en mm ---
figure;
hold on;

% Histograma suavizado (KDE)
[f, xi] = ksdensity(errors(:,1)); % Error en X
plot(xi, f, 'LineWidth', 2, 'DisplayName', 'Error en X');

[f, xi] = ksdensity(errors(:,2)); % Error en Y
plot(xi, f, 'LineWidth', 2, 'DisplayName', 'Error en Y');

[f, xi] = ksdensity(errors(:,3)); % Error en Z
plot(xi, f, 'LineWidth', 2, 'DisplayName', 'Error en Z');

% Etiquetas y leyenda
xlabel('Error (mm)');
ylabel('Densidad de Probabilidad');
title('Densidad de Probabilidad del Error en X, Y y Z');
legend('Location', 'Best');
grid on;
hold off;

% --- Graficar el error porcentual ---
% figure;
% hold on;
% 
% % Histograma suavizado (KDE) para error porcentual
% [f, xi] = ksdensity(percent_errors(:,1)); % Error porcentual en X
% plot(xi, f, 'LineWidth', 2, 'DisplayName', 'Error Porcentual en X');
% 
% [f, xi] = ksdensity(percent_errors(:,2)); % Error porcentual en Y
% plot(xi, f, 'LineWidth', 2, 'DisplayName', 'Error Porcentual en Y');
% 
% [f, xi] = ksdensity(percent_errors(:,3)); % Error porcentual en Z
% plot(xi, f, 'LineWidth', 2, 'DisplayName', 'Error Porcentual en Z');
% 
% % Etiquetas y leyenda
% xlabel('Error Porcentual (%)');
% ylabel('Densidad de Probabilidad');
% title('Densidad de Probabilidad del Error Porcentual en X, Y y Z');
% legend('Location', 'Best');
% grid on;
% hold off;

% Inicializar matrices para almacenar los promedios y desviaciones estándar
mean_errors = zeros(experiments, 3); % Para el promedio del error en X, Y, Z
std_errors = zeros(experiments, 3);  % Para la desviación estándar del error en X, Y, Z
mean_magnitude_error = zeros(experiments, 1); % Para el promedio de la magnitud del error
std_magnitude_error = zeros(experiments, 1);  % Para la desviación estándar de la magnitud del error

for j = 1:experiments
    % Obtener los errores de los 10 samples correspondientes al experimento j
    errors_j = errors((j-1)*samples + 1 : j*samples, :);

    % Calcular promedio y desviación estándar para X, Y y Z
    mean_errors(j, :) = mean(errors_j);
    std_errors(j, :) = std(errors_j);

    % Calcular magnitud del error para este experimento
    error_magnitude_j = sqrt(sum(errors_j.^2, 2));
    
    % Calcular el promedio y desviación estándar de la magnitud del error
    mean_magnitude_error(j) = mean(error_magnitude_j);
    std_magnitude_error(j) = std(error_magnitude_j);
end

% Mostrar resultados
disp('Promedio del error (en milímetros) por target point en X, Y, Z:');
disp(mean_errors);

disp('Desviación estándar del error (en milímetros) por target point en X, Y, Z:');
disp(std_errors);

disp('Promedio de la magnitud del error (en milímetros) por target point:');
disp(mean_magnitude_error);

disp('Desviación estándar de la magnitud del error (en milímetros) por target point:');
disp(std_magnitude_error);

% Calcular magnitud del error total
error_magnitude = sqrt(sum(errors.^2, 2)); % Error total en X, Y y Z

% Graficar KDE del error con áreas coloreadas
figure;
hold on;

% Definir los límites de las áreas coloreadas (por ejemplo, 0 a 3 mm, 3 a 6 mm, etc.)
ranges = [0, 3, 6, 9, 12]; % Limites en mm

% Histograma suavizado (KDE)
[f, xi] = ksdensity(error_magnitude); % Convertir a mm

% Graficar las áreas coloreadas
colors = [0.7 0.9 0.7; 0.4 0.7 0.9; 0.9 0.7 0.7; 0.7 0.7 0.9];
for i = 1:length(ranges)-1
    % Encontrar los índices de xi que están dentro del rango
    idx = xi >= ranges(i) & xi < ranges(i+1);
    
    % Rellenar el área correspondiente
    fill([xi(idx), fliplr(xi(idx))], [f(idx), zeros(1, sum(idx))], colors(i,:), 'FaceAlpha', 0.5, 'EdgeColor', 'none');
end

% Graficar KDE con línea gruesa
plot(xi, f, 'k', 'LineWidth', 2);

% Etiquetas y leyenda
xlabel('Magnitud del Error (mm)');
ylabel('Densidad de Detección (KDE)');
title('Densidad de Detección y Probabilidad del Error en X, Y y Z');
legend('0-3 mm', '3-6 mm', '6-9 mm', '9-12 mm', 'KDE', 'Location', 'Best');

% Anotar promedio y desviación estándar en la gráfica
for j = 1:experiments
    % Convertir el promedio y desviación estándar a mm
    mean_magnitude_mm = mean_magnitude_error(j);
    std_magnitude_mm = std_magnitude_error(j);

    % Añadir anotación a la gráfica
    text(mean_magnitude_mm, max(f)*0.8, ...
        ['Target ', num2str(j), ': \mu = ', num2str(mean_magnitude_mm, '%.2f'), ' mm, \sigma = ', num2str(std_magnitude_mm, '%.2f')], ...
        'FontSize', 10, 'BackgroundColor', 'w', 'EdgeColor', 'k');
end

% Ajustar ejes
xlim([0, max(ranges)]);
ylim([0, max(f) * 1.1]);

grid on;
hold off;

% --- Graficar la densidad de probabilidad del error en mm ---
figure;
hold on;

% Histograma suavizado (KDE) para el error en X
[f_x, xi_x] = ksdensity(errors(:,1)); % Error en X en mm
plot(xi_x, f_x, 'LineWidth', 2, 'DisplayName', 'Error en X');

% Histograma suavizado (KDE) para el error en Y
[f_y, xi_y] = ksdensity(errors(:,2)); % Error en Y en mm
plot(xi_y, f_y, 'LineWidth', 2, 'DisplayName', 'Error en Y');

% Histograma suavizado (KDE) para el error en Z
[f_z, xi_z] = ksdensity(errors(:,3)); % Error en Z en mm
plot(xi_z, f_z, 'LineWidth', 2, 'DisplayName', 'Error en Z');

% Añadir estadísticas a la gráfica (promedio y desviación estándar)
mean_error_x = mean(errors(:,1)) ; % Convertir a mm
mean_error_y = mean(errors(:,2)) ;
mean_error_z = mean(errors(:,3)) ;

std_error_x = std(errors(:,1)) ; % Convertir a mm
std_error_y = std(errors(:,2)) ;
std_error_z = std(errors(:,3)) ;

% Añadir anotaciones de promedio y desviación estándar a la gráfica
% Se ajustan las posiciones de las anotaciones para evitar superposición

text(6, 1, ...
    ['X: \mu = ', num2str(mean_error_x, '%.2f'), ' mm, \sigma = ', num2str(std_error_x, '%.2f')], ...
    'FontSize', 10, 'BackgroundColor', 'w', 'EdgeColor', 'k', 'HorizontalAlignment', 'center');

text(6, 0.8, ...
    ['Y: \mu = ', num2str(mean_error_y, '%.2f'), ' mm, \sigma = ', num2str(std_error_y, '%.2f')], ...
    'FontSize', 10, 'BackgroundColor', 'w', 'EdgeColor', 'k', 'HorizontalAlignment', 'center');

text(6, 0.6, ...
    ['Z: \mu = ', num2str(mean_error_z, '%.2f'), ' mm, \sigma = ', num2str(std_error_z, '%.2f')], ...
    'FontSize', 10, 'BackgroundColor', 'w', 'EdgeColor', 'k', 'HorizontalAlignment', 'center');

% Etiquetas y leyenda
xlabel('Error (mm)');
ylabel('Densidad de Probabilidad');
title('Densidad de Probabilidad del Error en X, Y y Z');
legend('Location', 'Best');
grid on;
grid minor
hold off;

