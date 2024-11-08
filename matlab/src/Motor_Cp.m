% Asume que tienes la matriz de inercia M y la matriz k_p definidas
k_p = [4.3 4.5 3.4 2 2.5 0.02];
robot = importrobot('archie_description\urdf\manipulator2.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];
jointVal = [0 0 0 0 0 0];
M = massMatrix(robot, jointVal); % Define la matriz de inercia del sistema

xi_desired = 0.7;
for i = 1:length(k_p)
    c_p(i) = 2 * xi_desired * sqrt(k_p(i) * M(i, i)); 
end

disp('Nuevos valores de Cp:');
disp(c_p)