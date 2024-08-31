robot = importrobot('archie_description\urdf\manipulator2.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

jointVal = [0 0 0 0 0 0];


k_p = eye(6,6).*[3 3.5 2.5 2 2 2]
c_p = eye(6,6).*[0.2 0.6 0.15 0.4 0.1 0.1]

M = massMatrix(robot, jointVal)
M_inv = inv(M)

A = [zeros(6,6), eye(6,6); %         Zeros(6x6), I(6x6)
     M_inv*(-k_p),  M_inv*(-c_p)];%-M_inv*K  , -M_inv*C 

eigen_val = eig(A)

wn = [];
damp = [];
for i=1:12
    wn = [wn;abs(eigen_val(i))];
    damp = [damp; cos(angle(eigen_val(i)))];
end

wn
damp
