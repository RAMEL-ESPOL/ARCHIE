robot = importrobot('archie_description\urdf\manipulator.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];


jointVal = [0 0 0 0 0 0];

M = massMatrix(robot, jointVal) % Define la matriz de inercia del sistema

J = geometricJacobian(robot, jointVal,'link_6')



% Diferentes valores de PD y PID usados hasta ahora
% PID1
k_p = [3 3.5 2.5 2 2 0];
c_p = [0.1977    0.6622    0.3440    0.0615    0.0679    0.0026];
i_p = [0 0 0 0 0 0];


% PID2
k_p = [2 2.5 1.5 1 1 1];
c_p = [0.2451    0.3936    0.2219    0.0530    0.0421    0.0016];
i_p = [0 0 0 0 0 0];

% PID3
k_p = [3.7 4 3 2 2 0.02];
c_p = [0.3333    0.4978    0.3138    0.0749    0.0595    0.0009];
i_p = [0 0 0 0 0 0];

% PID4
k_p = [3.9 4.2 3 2 2 0.02];
c_p = [0.3333    0.4978    0.3138    0.0749    0.0595    0.0009];
i_p = [0.1 0.1 0.1 0.1 0.1 0.1];

% PID5
k_p = [3.9 4.2 3 2 2 0.02];
c_p = [0.3333    0.4978    0.3138    0.0749    0.0595    0.0009];
i_p = [0.1 5 5 0.1 0.1 0.1];

% PID6
k_p = [4.3 4.5 3.4 2 2.5 0.02];
c_p = [0.3593    0.5280    0.3341    0.0749    0.0665    0.0002];
i_p = [0.1 10 5 0.1 0.1 0.1];

% PID7
k_p = [4.3 4.5 3.4 2 2.5 0.02];
c_p = [0.3593    0.58    0.38    0.0749    0.0665    0.0002];