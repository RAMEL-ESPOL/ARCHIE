robot = importrobot('archie_description\urdf\manipulator2.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

samples = 10;
errors = zeros(samples, 3); % Para almacenar errores en X, Y y Z
percent_errors = zeros(samples, 3); % Para almacenar errores porcentuales

for i = 1:samples
    % Construir el nombre del archivo
    filename_real = strcat('matlab/data/', num2str(i), '_joint_real_states_precision_x15_y15_z20.txt');
    x_target = 0.15;
    y_target = 0.15;
    z_target = 0.2;

    % Leer estados reales
    joint_states = deg2rad(table2array(readtable(filename_real)));
    tform_states = getTransform(robot, joint_states(end, : ), 'link_6'); 
    pos_states = tform2trvec(tform_states);

    % Calcular error
    error_x = pos_states(1) - x_target;
    error_y = pos_states(2) - y_target;
    error_z = pos_states(3) - z_target;

    % Guardar error absoluto en mm
    errors(i, :) = [error_x, error_y, error_z] * 1000;

    % Calcular error porcentual
    percent_errors(i, :) = [error_x / x_target, error_y / y_target, error_z / z_target] * 100;
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
grid minor;
hold off;

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

text(-5, 0.6, ...
    ['X: \mu = ', num2str(mean_error_x, '%.2f'), ' mm, \sigma = ', num2str(std_error_x, '%.2f')], ...
    'FontSize', 10, 'BackgroundColor', 'w', 'EdgeColor', 'k', 'HorizontalAlignment', 'center');

text(-5, 0.56, ...
    ['Y: \mu = ', num2str(mean_error_y, '%.2f'), ' mm, \sigma = ', num2str(std_error_y, '%.2f')], ...
    'FontSize', 10, 'BackgroundColor', 'w', 'EdgeColor', 'k', 'HorizontalAlignment', 'center');

text(-5, 0.52, ...
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

function T = plot_dist(filename_real, x_target, y_target, z_target)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end