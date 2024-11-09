ndoc = 11;
joint_error_real = importdata(strcat('matlab/data_pwm/',num2str(ndoc),'_motor_error.txt'));joint_states_real = importdata(strcat('matlab/data_pwm/',num2str(ndoc),'_motor_position.txt'));joint_goals = joint_error_real + joint_states_real;

time_real= [];

for i=1:length(joint_goals)
    time_real = [time_real, i*0.1];
end

simu_decimation = 1;
step_time = 0.1;
stop_time = time_real(length(time_real));


input_data = [time_real', joint_goals];

robot = importrobot('archie_description\urdf\manipulator.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];




% PID1
k_p = [3 3.5 2.5 2 2 0];
c_p = [0.1977    0.6622    0.3440    0.0615    0.0679    0.0026];
i_p = [0 0 0 0 0 0];




out = sim("matlab\src\Simulink\manipulator_pid_torque", 'StopTime', num2str(stop_time));
