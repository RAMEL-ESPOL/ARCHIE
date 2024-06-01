% Importar el robot desde el archivo URDF
robot = importrobot('manipulator_final2/urdf/manipulator_final2.urdf');
robot.DataFormat = 'row';

% % Mostrar el robot para asegurarnos de que se ha importado correctamente
% show(robot);

% Definir los límites de las articulaciones
jointLimits = [-pi/2 pi/2; -0.78 0.78; -1.15 2; -pi pi; -1.15 2; -pi pi]; % Ajusta según tu robot

% Número de muestras por articulación
% En este momento el código tarda mucho en ejecutarse por lo que lo mejor es
% usar un menor numSamples en caso de que se vuelva a correr el código
numSamples = 20;

% Generar un conjunto de configuraciones de articulaciones
jointConfigurations = [];

for i = 1:size(jointLimits, 1)
    jointConfigurations = [jointConfigurations; linspace(jointLimits(i, 1), jointLimits(i, 2), numSamples)];
end

% Calcular las posiciones del efector final
positions = [];
for i1 = 1:numSamples
    for i2 = 1:numSamples
        for i3 = 1:numSamples
            for i4 = 1:10
                for i5 = 1:numSamples
                    % Configuración de las articulaciones
                    jointConfig = [jointConfigurations(1, i1), jointConfigurations(2, i2), ...
                                   jointConfigurations(3, i3), jointConfigurations(4, i4), ...
                                   jointConfigurations(5, i5), jointConfigurations(6, 1)];
                    % Calcular la posición del efector final
                    tform = getTransform(robot, jointConfig, 'link_6'); % Reemplaza 'end_effector_name' con el nombre real de tu efector final
                    pos = tform2trvec(tform);
                    positions = [positions; pos];
                end
            end
        end
    end
end

% Visualizar las posiciones del efector final
figure;
scatter3(positions(:,1), positions(:,2), positions(:,3), 'filled');
title('Espacio de Trabajo del Robot');
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;