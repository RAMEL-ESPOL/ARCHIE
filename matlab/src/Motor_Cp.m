% Asume que tienes la matriz de inercia M y la matriz k_p definidas
kp = [3, 3.5, 2.5, 2, 2, 2];
M = massMatrix(robot, jointVal); % Define la matriz de inercia del sistema

xi_desired = 0.7;
for i = 1:length(kp)
    cp(i) = 2 * xi_desired * sqrt(kp(i) * M(i, i)); 
end

disp('Nuevos valores de Cp:');
disp(cp);


