function [A, H, Q] = function_dyn(q1, q2, q1d, q2d, theta)
%% My parameters

global I;
global s;

l = 0.8; % [m]
m = 2; % [Kg]
g = 9.8; % [Kg*s^(-2)]



%% Matrix A

A11 = m*(l-s)^2 + 2*I + m*l^2 + m*s^2 + 2*m*l*s*cos(q2);
A12 = m*s^2 + I + m*l*s*cos(q2);
A21 = m*s^2 + I + m*l*s*cos(q2);
A22 = m*s^2 + I;
A = [A11 A12 ; A21 A22];


%% Vector H
% Knowing that H = B*q1d*q2d + C*qd^2

% Define vector qd

qd = [q1d ; q2d];


% Define matrix B 

B11 = -2*m*l*s*sin(q2);
B21 = 0;

B = [B11 ; B21];


% Define matric C

C11 = 0;
C12 = -m*l*s*sin(q2);
C21 = m*l*s*sin(q2);
C22= 0;

C = [C11 C12 ; C21 C22];


% Define vector H

H = B*q1d*q2d + C*qd.^2;


%% Define Q

% % Define Gravity vector
% % Defined it in 2D to match the position's vectors of G1 and G2
% grav = [g*sin(theta); -g*cos(theta)];
% 
% % Define the position vectors of G1 and G2
% 
% G1 = (l-s)*[-sin(q1) ; cos(q1)]; % For leg 1
% G2 = l*[-sin(q1) ; cos(q1)] + s*[-sin(q1+q2) ; cos(q1+q2)];
% 
% 
% % Define potential energies
% 
% U1 = -m * grav.' * G1;
% U2 = -m * grav.' * G2;
% U = U1 + U2;


% Define Q

Q1 = m*g*(l-s)*sin(theta-q1)+m*g*l*sin(theta-q1)+m*g*s*sin(theta-(q1+q2));
Q2 = m*g*s*sin(theta-(q1+q2));
Q = [Q1 ; Q2];




























end




