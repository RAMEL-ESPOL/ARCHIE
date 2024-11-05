% joint_goals   = deg2rad(table2array(readtable('matlab/data/joint_goals_square_t5_h30_p22.txt')));

% joint_goals = zeros(30000, 6);
joint_goals_step = [-1.57 1.57/2 -1.57/2 1.57/3 1.57 1.57];
% input_data = [(linspace(0, 5, length(joint_goals)))', joint_goals];

robot = importrobot('archie_description\urdf\manipulator2.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

k_p = [3 3.5 2.5 2 2 0];
c_p = [0.1977    0.6622    0.3440    0.0615    0.0679    0.0026];


out = sim("manipulator_pid_torque");

