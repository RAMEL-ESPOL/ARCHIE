% Código para graficar la respuesta a escalón de un sistema de segundo orden
clear; clc; close all;

% Parámetros del sistema
Kp = 2;       % Ganancia proporcional
Cp = 0.7;     % Coeficiente de amortiguamiento (0 < Cp < 1 para subamortiguado)
Ip = 2;       % Frecuencia natural (rad/s)

% Definir el sistema de segundo orden (función de transferencia)
num = [Kp * Ip^2];
den = [1, 2*Cp*Ip, Ip^2];
sys = tf(num, den);

% Simulación de la respuesta al escalón
t = 0:0.1:35; % Tiempo de simulación
[y, t] = step(sys, t);

% Graficar la respuesta al escalón
figure;
hold on;
plot(t, y, 'r', 'LineWidth', 1.5); % Respuesta del sistema
plot(t, ones(size(t)), 'g', 'LineWidth', 1); % Línea de referencia (entrada escalón)
grid on;grid minor;

% Personalización de la gráfica
title('Step Response', 'FontSize', 14);
xlabel('Time (seconds)', 'FontSize', 12);
ylabel('Amplitude', 'FontSize', 12);
legend('System Response', 'Step Input', 'Location', 'SouthEast');
xlim([0 35]);

% Ajustar propiedades
set(gca, 'FontSize', 10, 'GridAlpha', 0.4);

% Indicación para modificar parámetros
disp('Puedes modificar Kp, Cp, e Ip al inicio del código para ajustar la respuesta del sistema.');
