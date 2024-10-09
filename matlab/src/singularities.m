robot = importrobot('archie_description\urdf\manipulator2.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

joint_min = [-pi/4, -pi/4, -pi/2, -pi/3, -pi/2, 0]; % mínimos de cada joint
joint_max = [ pi/4,  pi/4,  pi/2,  pi/2,  pi/2, 0]; % máximos de cada joint
resolucion = 0.8; % paso de iteración

jointArray = [];
detJointArray = [];
minDetArray = [];
minJointArray = [];
for q1 = joint_min(1):resolucion:joint_max(1)
    for q2 = joint_min(2):resolucion:joint_max(2)
        detArray = [];
        for q3 = joint_min(3):resolucion:joint_max(3)
            for q4 = joint_min(4):resolucion:joint_max(4)
                
                for q5 = joint_min(5):resolucion:joint_max(5)
                    jointVal = [q1, q2, q3, q4, q5, 0];
                    M = massMatrix(robot, jointVal);
                    detM = det(M);
                    detArray = [detArray; detM];

                    if abs(detM) < 1.5e-18 % Condición de singularidad aproximada
                        disp('Matriz de inercia cerca de ser singular en:')
                        disp(jointVal)
                        minJointArray = [minJointArray ; rad2deg(jointVal)];
                        minDetArray = [minDetArray; detM];
%                         break
                    end
                    jointArray = [jointArray; rad2deg(jointVal)];
                    detJointArray = [detJointArray; detM];
                end
            end
        end
        figure();
        plot(detArray,"LineWidth",2)
        grid on
        grid minor

    end
end
figure();
plot(detJointArray,"LineWidth",2)
grid on
grid minor

figure();
plot(minDetArray,"LineWidth",2)
grid on
grid minor


% Visualizar el robot en las configuraciones donde la matriz es cercana a singular
% figure;
% grid on 
% grid minor
% hold on
% axis equal
% for i = 1:size(minJointArray, 1)
%     config = deg2rad(minJointArray(i, :)); % Convertir de grados a radianes
%     show(robot, config, 'PreservePlot', false); % Visualizar el robot
%     pause(0.5); % Pausa para ver cada configuración
% end
% hold off

% show(robot, jointArray(90, :))


% % Selecciona dos articulaciones para graficar
% joint_min = [-pi/2, -pi/2];
% joint_max = [pi/2, pi/2];
% resolucion = 0.1;
% 
% [q1_vals, q2_vals] = meshgrid(joint_min(1):resolucion:joint_max(1), ...
%                               joint_min(2):resolucion:joint_max(2));
% det_vals = zeros(size(q1_vals));
% 
% for i = 1:size(q1_vals, 1)
%     for j = 1:size(q1_vals, 2)
%         jointVal = [q1_vals(i, j), q2_vals(i, j), 0, 0, 0, 0]; % Fijar el resto de los joints
%         M = massMatrix(robot, jointVal);
%         det_vals(i, j) = det(M);
%     end
% end
% 
% figure;
% surf(q1_vals, q2_vals, det_vals)
% xlabel('Joint 1')
% ylabel('Joint 2')
% zlabel('Determinante de M')
% title('Determinante de la matriz de inercia')

