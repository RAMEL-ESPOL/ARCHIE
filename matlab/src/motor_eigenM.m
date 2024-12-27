robot = importrobot('archie_description\urdf\manipulator2.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

n = 6;
jointVal = rand(1,n)

jointVal = [0,0,0,0,0,0];

k_p = eye(n,n).*[3 3.5 2.5 2 2 2];
c_p = eye(n,n).*[0.2 0.6 0.15 0.4 0.1 0.1]; Antiguo
% c_p = eye(n,n).*[0.1740    0.5546    0.2957    0.0673    0.0595    0.0023];
c_p = eye(n,n).*[0.1697    0.4348    0.2545    0.0558    0.0509    0.0020];
% c_p = eye(n,n).*[0.2269    0.4579    0.2234    0.0581    0.0467    0.0018];
c_p = eye(n,n).*[0.1977    0.6622    0.3440    0.0615    0.0679    0.0026];

M = massMatrix(robot, jointVal);
M_inv = inv(M)

%Matriz de estados (revisar)
A = [zeros(n,n), eye(n,n); %         Zeros(nxn), I(nxn)
     M_inv*(-k_p),  M_inv*(-c_p)];%  -M_inv*K  , -M_inv*C 

eigen_val = eig(A)

wn = [];
damp = [];
for i=1:length(eigen_val)
    wn = [wn;abs(eigen_val(i))];
    damp = [damp; cos(angle(eigen_val(i)))];
end

[wn damp*10000]