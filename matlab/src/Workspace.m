% Definir el robot usando la función importrobot de Robotics System Toolbox
robot = importrobot('manipulator_description/urdf/manipulator.urdf');
robot.DataFormat = 'row';

% Definir las posiciones articulares (en radianes) para el cálculo del workspace
jointPositions = zeros(1, 6); % Por ejemplo, todas las articulaciones en posición inicial
jointVelocities = zeros(1, 6); % Velocidades iniciales, en radianes por segundo

% Obtener los nombres de las articulaciones
for i = 1:6
    bodyNames(i) = getBody(robot,strcat("link_", string(i)));
end

% Inferir los nombres de los cuerpos asociados
bodyNames = cell(1, length(bodyNames));
for i = 1:length(bodyNames)
    jointNames(i) = strrep(bodyNames(i), 'link_', 'joint_');
end

% Configurar el robot en la configuración dada
for i = 1:length(jointPositions)
    robot2 = setJointState(robot, jointNames{i}, 'position', jointPositions(i), 'velocity', jointVelocities(i));
end

% Calcular la posición del extremo del robot (end effector)
tform = getTransform(robot2, bodyNames{end}, 'base_link');

% Extraer la posición del extremo del robot
endEffectorPosition = tform(1:3, 4)';

% Mostrar la posición del extremo del robot en coordenadas XYZ
disp('Posición del extremo del robot (en coordenadas XYZ):');
disp(endEffectorPosition);