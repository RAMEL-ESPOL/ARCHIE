% Crear una figura nueva
figure;

% Graficar los puntos del workspace
scatter3(positions(:,1), positions(:,2), positions(:,3), 'filled', ...
    'MarkerEdgeAlpha', 0.15, ... % Transparencia del borde del marcador
    'MarkerFaceAlpha', 0.15);    % Transparencia de la cara del marcador

% Añadir etiquetas y título
xlabel('Axis X (m)');
ylabel('Axis Y (m)');
zlabel('Axis Z (m)');
title('Workspace of the robotic arm');

% Configuración adicional para mejorar la visualización
grid on;
axis equal;
view(3);

% Supongamos que también tienes el robot en la misma figura
robot = importrobot('archie_description\urdf\archie_without_pen.urdf');
robot.DataFormat = 'row';
hold on;
show(robot);

% Mejorar la transparencia del robot para que no bloquee los puntos
h = findobj(gca, 'Type', 'Patch');
set(h, 'FaceAlpha', 0.95); % Ajustar la transparencia del robot

% Mantener la gráfica
hold off;
