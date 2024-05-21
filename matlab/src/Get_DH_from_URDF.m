% Cargar el modelo URDF del robot
robot = importrobot('manipulator_final2\urdf\manipulator_final2.urdf');
show(robot); % Visualizar el robot para verificar la carga

% Obtener todas las articulaciones del robot
joints = robot.homeConfiguration;
nJoints = length(joints);

% Inicializar los parámetros DH
DH_params = zeros(nJoints, 4); % [theta, d, a, alpha]

for i = 1:nJoints
    % Obtener la transformación de la articulación i
    jointName = joints(i).JointName;
    joint = robot.getBody('link_').Joint;
    
    % Extraer las transformaciones de la articulación
    T_joint = joint.JointToParentTransform;
    R = tform2rotm(T_joint); % Matriz de rotación
    p = tform2trvec(T_joint); % Vector de traslación
    
    % Convertir a parámetros DH
    DH_params(i, :) = tr2dhparams(R, p);
end

function dh = tr2dhparams(R, p)
    % tr2dhparams - Convertir una matriz de transformación a parámetros DH
    % R: Matriz de rotación 3x3
    % p: Vector de traslación 1x3

    % Inicializar los parámetros DH
    theta = 0;
    d = 0;
    a = 0;
    alpha = 0;

    % Calcular theta
    theta = atan2(R(2, 1), R(1, 1));

    % Calcular d
    d = p(3);

    % Calcular a
    a = sqrt(p(1)^2 + p(2)^2);

    % Calcular alpha
    alpha = atan2(R(3, 2), R(3, 3));

    % Asignar los parámetros DH
    dh = [theta, d, a, alpha];
end
