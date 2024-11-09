joint_goals   = deg2rad(table2array(readtable('matlab/data/joint_goals_square_t5_h30_p22.txt')));

simu_decimation = 2;
step_time = 0.005;
stop_time = 5;
% joint_goals = zeros(30000, 6);
joint_goals_step = [-1.57 1.57/2 -1.57/2 1.57/3 1.57 1.57];
input_data = [(linspace(0, stop_time, length(joint_goals)))', joint_goals];

robot = importrobot('archie_description\urdf\manipulator.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

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
i_p = [0 0 0 0 0 0];

% PID5
k_p = [4.3 4.5 3.4 2 2.5 0.02];
c_p = [0.3593    0.5280    0.3341    0.0749    0.0665    0.0002];
i_p = [0 0 0 0 0 0];

out = sim("matlab\src\Simulink\manipulator_pid_torque", 'StopTime', num2str(stop_time));
