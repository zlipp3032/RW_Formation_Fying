% Evaluate the gains in the experiment
clear
clc
close all

% Inter-agent position gains
a12 = 0.1;
a21 = 0.2;


% Inter-agent velocity gains
b12 = 0.433;
b21 = 0.866;


% Leader Position gains
g1 = 0.2;
g2 = 0.0;

% Leader Velocity gains
e1 = 0.866;
e2 = 0.0;


% Set the adjacency matrix
A = [0 a12; a21 0];
B = [0 b12; b21 0];

% Graph Laplacians
La = diag(sum(A,2)) - A;
Lb = diag(sum(B,2)) - B;

% Leader Matrices
G = diag([g1;g2]);
E = diag([e1;e2]);

% Leader weighted Laplacians
La_hat = La+G;
Lb_hat = Lb+E;



% Set the close-loop system matrix
Acl = [zeros(2,2) eye(2,2); -La_hat -Lb_hat];

CL_eig = eig(Acl)

var = (real(CL_eig).^2)./(imag(CL_eig).^2);
zeta = (var./sqrt(1+var.^2))'

T_settle = -4./real(CL_eig)'





