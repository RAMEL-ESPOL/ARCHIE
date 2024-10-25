joint_goals   = deg2rad(table2array(readtable('matlab/data/joint_goals_square_t5_h30_p22.txt')));
t = linspace(0, 10, 327);
input_data = [t', zeros(327, 6)];

robot = importrobot('archie_description\urdf\manipulator.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

n = 6;
k_p = eye(n,n).*[3 3.5 2.5 2 2 0];
c_p = eye(n,n).*[0.1697    0.4348    0.2545    0.0558    0.0509    0.0];


out = sim("manipulator_torque");