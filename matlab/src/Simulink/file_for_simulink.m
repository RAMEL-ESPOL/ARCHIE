joint_goals   = deg2rad(table2array(readtable('matlab/data/joint_goals_square_t5_h30_p22.txt')));

simu_decimation = 1;
step_time = 0.1;
stop_time = 3;
n_pid = 6;

% joint_goals = zeros(30000, 6);
joint_goals_step = [-1.57 1.57/2 -1.57/2 1.57/3 1.57 1.57];
input_data = [(linspace(0, stop_time, length(joint_goals)))', joint_goals];

robot = importrobot('archie_description\urdf\manipulator.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

% Con esto únicamente debemos cambiar la variable ndoc para usar otro array de PID
% lo que nos evita tener que andar comentando y/o descomentando código
pid_array(:,:,1) = [
    3 3.5 2.5 2 2 0;
    0.1977    0.6622    0.3440    0.0615    0.0679    0.0026;
    0 0 0 0 0 0
];
pid_array(:,:,2) = [
    2 2.5 1.5 1 1 1;
    0.2451    0.3936    0.2219    0.0530    0.0421    0.0016;             
    0 0 0 0 0 0
];
pid_array(:,:,3) = [
    3.7 4 3 2 2 0.02;
    0.3333    0.4978    0.3138    0.0749    0.0595    0.0009;             
    0 0 0 0 0 0
];
pid_array(:,:,4) = [
    3.9 4.2 3 2 2 0.02;
    0.3333    0.4978    0.3138    0.0749    0.0595    0.0009;             
    0 0 0 0 0 0
];
pid_array(:,:,5) = [
    4.3 4.5 3.4 2 2.5 0.02;
    0.3593    0.5280    0.3341    0.0749    0.0665    0.0002;
    0 0 0 0 0 0
];

pid_array(:,:,6) = [
    1 1 1 1 1 1;
    0 0 0 0 0 0;
    0 0 0 0 0 0
];



k_p = pid_array(1,:,n_pid);
c_p = pid_array(2,:,n_pid);
i_p = pid_array(3,:,n_pid);


out = sim("matlab\src\Simulink\manipulator_pid_torque", 'StopTime', num2str(stop_time));
