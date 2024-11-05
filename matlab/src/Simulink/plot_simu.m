torque= out.final_torques;
damp = out.damp_torques;
% damp = zeros(length(torque), 6);
stiff = out.stiff_torques;

colors = lines(3);

figure(1);
title("Torques");
subplot(3,2,1)
plot(torque(:,1), 'LineWidth', 2, 'Color', colors(1,:));
grid on
grid minor
hold on
plot(damp(:,1), 'LineWidth', 2, 'Color', colors(2,:));
hold on 
plot(stiff(:,1), 'LineWidth', 2, 'Color', colors(3,:));


subplot(3,2,2)
plot(torque(:,2), 'LineWidth', 2, 'Color', colors(1,:));
grid on
grid minor
hold on
plot(damp(:,2), 'LineWidth', 2, 'Color', colors(2,:));
hold on 
plot(stiff(:,2), 'LineWidth', 2, 'Color', colors(3,:));


subplot(3,2,3)
plot(torque(:,3), 'LineWidth', 2, 'Color', colors(1,:));
grid on
grid minor
hold on
plot(damp(:,3), 'LineWidth', 2, 'Color', colors(2,:));
hold on 
plot(stiff(:,3), 'LineWidth', 2, 'Color', colors(3,:));


subplot(3,2,4)
plot(torque(:,4), 'LineWidth', 2, 'Color', colors(1,:));
grid on
grid minor
hold on
plot(damp(:,4), 'LineWidth', 2, 'Color', colors(2,:));
hold on 
plot(stiff(:,4), 'LineWidth', 2, 'Color', colors(3,:));


subplot(3,2,5)
plot(torque(:,5), 'LineWidth', 2, 'Color', colors(1,:));
grid on
grid minor
hold on
plot(damp(:,5), 'LineWidth', 2, 'Color', colors(2,:));
hold on 
plot(stiff(:,5), 'LineWidth', 2, 'Color', colors(3,:))


subplot(3,2,6)
plot(torque(:,6), 'LineWidth', 2, 'Color', colors(1,:));
grid on
grid minor
hold on
plot(damp(:,6), 'LineWidth', 2, 'Color', colors(2,:));
hold on 
plot(stiff(:,6), 'LineWidth', 2, 'Color', colors(3,:))
legend("Final Torque", "Damp Torque", "Stiff Torque")