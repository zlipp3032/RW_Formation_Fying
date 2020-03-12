% Evaluate the gains in the experiment
clear
clc
close all

% Inter-agent position gains
a12 = 0;
a13 = 0;
a21 = 0.2;
a23 = 0;
a31 = 0.2;
a32 = 0;


% Inter-agent velocity gains
b12 = 0;
b13 = 0;
b21 = 0.866;
b23 = 0;
b31 = 0.866;
b32 = 0;


% Leader Position gains
g1 = 0.3;
g2 = 0.0;
g3 = 0.0;

% Leader Velocity gains
e1 = 1.3;
e2 = 0.0;
e3 = 0.0;


% Set the adjacency matrix
A = [0 a12 a13; a21 0 a23; a31 a32 0];
B = [0 b12 b13; b21 0 b23; b31 b32 0];

% Graph Laplacians
La = diag(sum(A,2)) - A;
Lb = diag(sum(B,2)) - B;

% Leader Matrices
G = diag([g1;g2;g3]);
E = diag([e1;e2;e3]);

% Leader weighted Laplacians
La_hat = La+G;
Lb_hat = Lb+E;



% Set the close-loop system matrix
Acl = [zeros(3,3) eye(3,3); -La_hat -Lb_hat];

CL_eig = eig(Acl)

var = (real(CL_eig).^2)./(imag(CL_eig).^2);
zeta = (var./sqrt(1+var.^2))'

T_settle = -4./real(CL_eig)'





