% Crear variables simbólicas para las articulaciones
syms q1 q2 q3 q4 q5 q6 real
jointVars = [q1 q2 q3 q4 q5 q6];

% Parámetros DH: [theta d a alpha]
DH_params = [
    q1+pi/2, 0.09, 0.02, pi/2;
    q2+pi/2, 0.00, 0.20, 0.0 ;
    q3     , 0.00, 0.02, pi/2;
    q4     , 0.23,-0.02,-pi/2;
    q5-pi/2, 0.00, 0.00, pi/2;
    q6     , 0.04, 0.00, 0.0 ;
];

% Inicializar la transformación total como la identidad
T = eye(4);

% Crear una celda para almacenar las transformaciones homogéneas
T_matrices = cell(1, size(DH_params, 1));

% Recorrer cada fila de los parámetros DH para calcular las transformaciones
for i = 1:size(DH_params, 1)
    theta = DH_params(i, 1);
    d = DH_params(i, 2);
    a = DH_params(i, 3);
    alpha = DH_params(i, 4);
    
    T = T * dh_transform(theta, d, a, alpha);
    T_matrices{i} = T;
end

% La última transformación T contiene la posición del efector final
T_efector = T;

% Obtener la posición del efector final
pos_efector = T_efector(1:3, 4);

% Inicializar la matriz jacobiana
J = sym(zeros(6, 6));

for i = 1:6
    % (z_i-1) X (P_eff - P_i-1)
    %  z_i-1
    if i > 1
        R_efector = T_matrices{i-1}(1:3, 1:3);
        J(1:3, i) = cross(R_efector(:, 3), pos_efector - T_matrices{i-1}(1:3,4));
        J(4:6, i) = R_efector(:, 3);
    else
        J(1:3, i) = cross(transpose([0 0 1]), pos_efector - transpose([0 0 0]));
        J(4:6, i) = transpose([0 0 1]);
    end
end

%Simplificar la matriz jacobiana
J = simplify(J);

% Mostrar el Jacobiano simbólico
disp('Jacobiano simbólico:');
disp(J);

disp('Trasformada simbólica:');
disp(T_efector);
% Calcular el determinante del Jacobiano
det_J = simplify(det(J));

% Mostrar el determinante simbólico
disp('Determinante del Jacobiano simbólico:');
disp(det_J);

jointVal = [0 0 0 0 0 0];

robot = importrobot('manipulator_final2\urdf\manipulator_final2.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

J_home = geometricJacobian(robot,[0 0 0 0 0 0],'link_6');
J3 = J_home(1:3, :);
J_home(1:3, :) = J_home(4:6, :);
J_home(4:6,:) = J3;

% Mostrar el resultado
disp('Jacobiano sacado de DH Parameters usando la configuración inicial:');
disp(double(subs(J, jointVars, jointVal)));
disp('Jacobiano del urdf usando la configuración inicial:');
disp(J_home);
disp('Determinante sacado de DH Parameters usando la configuración inicial:');
disp(double(subs(det_J, jointVars, jointVal)));
disp('Determinante del urdf usando la configuración inicial:');
disp(det(J_home));

trans_urdf = getTransform(robot,jointVal,'link_6');
disp(trans_urdf);

disp('Transformada del efector final usando parámetros DH:')
disp(double(subs(T_efector, jointVars, jointVal)));
disp('Transformada del efector final usando urdf:')
disp(trans_urdf);

% Función para crear la matriz de transformación usando los parámetros DH
function T = dh_transform(theta, d, a, alpha)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end