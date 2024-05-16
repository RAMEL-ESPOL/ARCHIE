% Definir el robot usando la función importrobot de Robotics System Toolbox
robot = importrobot('manipulator_description/urdf/manipulator.urdf');

% Definir las posiciones articulares (en radianes) para el cálculo del workspace
jointPositions = zeros(1, 6); % Por ejemplo, todas las articulaciones en posición inicial
jointVelocities = zeros(1, 6); % Velocidades iniciales, en radianes por segundo

% Configurar el robot en la configuración dada
robot = homeConfiguration(robot);

% Obtener los nombres de las articulaciones
jointNames = {robot.JointName};

% Inferir los nombres de los cuerpos asociados
bodyNames = cell(1, length(jointNames));
for i = 1:length(jointNames)
    bodyNames{i} = strrep(jointNames{i}, 'joint_', 'link_');
end

% Configurar el robot en la configuración dada
for i = 1:length(jointPositions)
    robot = setJointState(robot, jointNames{i}, 'position', jointPositions(i), 'velocity', jointVelocities(i));
end

% Calcular la posición del extremo del robot (end effector)
tform = getTransform(robot, bodyNames{end}, 'base_link');

% Extraer la posición del extremo del robot
endEffectorPosition = tform(1:3, 4)';

% Mostrar la posición del extremo del robot en coordenadas XYZ
disp('Posición del extremo del robot (en coordenadas XYZ):');
disp(endEffectorPosition);