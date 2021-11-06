function [A1, J_R] = function_impact(q1, q2)
%% My parameters

global I;
global s;

l = 0.8; % [m]
m = 2; % [Kg]
g = 9.8; % [Kg*s^(-2)]


%% Define matrix A1


A11 = 2*m;
A12 = 0;
A13 = m*s*cos(q1) -m*s*cos(q1 + q2);
A14 = -m*s*cos(q1 + q2);
A21 = 0;
A22 = 2*m;
A23 = m*s*sin(q1) -m*s*sin(q1+q2);
A24 = -m*s*sin(q1+q2);
A31 = m*s*cos(q1) -m*s*cos(q1 + q2);
A32 = m*s*sin(q1) -m*s*sin(q1+q2);
A33 = 2*m*s^2 +2*I;
A34 = m*s^2 +I;
A41 = -m*s*cos(q1 + q2);
A42 = -m*s*sin(q1 + q2);
A43 = m*s^2 +I;
A44 = m*s^2 +I;

A1 = [A11 A12 A13 A14; A21 A22 A23 A24; A31 A32 A33 A34; A41 A42 A43 A44];


%% Define matrix J_R

J_R = [1  0  -l*cos(q1+q2) -l*cos(q1+q2); 0 1 -l*sin(q1+q2) -l*sin(q1+q2)];


end


