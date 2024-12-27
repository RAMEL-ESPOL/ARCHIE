robot = importrobot('archie_description\urdf\manipulator2.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

n = 6;
jointVal = rand(1,n)

jointVal = [0,0,0,0,0,0];

k_p = eye(n,n).*[3 3.5 2.5 2 2 2];
% c_p = eye(n,n).*[0.2 0.6 0.15 0.4 0.1 0.1]; Antiguo
% c_p = eye(n,n).*[0.1740    0.5546    0.2957    0.0673    0.0595    0.0023];
c_p1 = eye(n,n).*[0.1697    0.4348    0.2545    0.0558    0.0509    0.0020];
% c_p = eye(n,n).*[0.2269    0.4579    0.2234    0.0581    0.0467    0.0018];
c_p2 = eye(n,n).*[0.1977    0.6622    0.3440    0.0615    0.0679    0.0026];
M = massMatrix(robot, jointVal);
M_inv = inv(M)

%Matriz de estados (revisar)
A1 = [zeros(n,n), eye(n,n); %         Zeros(nxn), I(nxn)
     M_inv*(-k_p),  M_inv*(-c_p1)];%  -M_inv*K  , -M_inv*C 

A2 = [zeros(n,n), eye(n,n); %         Zeros(nxn), I(nxn)
     M_inv*(-k_p),  M_inv*(-c_p2)];%  -M_inv*K  , -M_inv*C 

% Obtener autovalores y autovectores
[V1, D1] = eig(A1); % D1 contiene los eigenvalores
[V2, D2] = eig(A2); % D2 contiene los eigenvalores

% Convertimos los valores propios en una matriz de autovalores (p x p x n)
eig_matrices(:,:,1) = A1;
eig_matrices(:,:,2) = A2;

% Usamos eigenshuffle para emparejar los eigenvalores
[eigen_val_A2_shuffled, vlues] = eigenshuffle(eig_matrices);

wn1   = [];
wn2   = [];
damp1 = [];
damp2 = [];

for i=1:length(vlues)
    wn1   = [wn1;abs(vlues(i, 1))];
    wn2   = [wn2;abs(vlues(i, 2))];
    damp1 = [damp1; cos(angle(vlues(i, 1)))];
    damp2 = [damp2; cos(angle(vlues(i, 2)))];
end

[wn1 wn2]
disp("");
[damp1 damp2]
