function [F] = function_reactionforce(q1, q2, q1d, q2d, q1dd, q2dd)
%% My parameters

l = 0.8; % [m]
m = 2; % [Kg]
I = 0.1; % [Kg*m^2]
s = 0.5; % [m]
g = 9.8; % [Kg*s^(-2)]
theta = 2*pi/180;


%% Reaction force
% F = 2*m*Gdd - 2*m*grav

%  Define Gravity vector

grav = [g*sin(theta); -g*cos(theta)];


% Define my Gdd, which is the acceleration of mass center 

Gddx = l*sin(q1)*q1d^2 -l*cos(q1)*q1dd +0.5*s*cos(q1)*q1dd -0.5*s*sin(q1)*q1d^2 +0.5*s*sin(q1+q2)*(q1d+q2d)^2 -0.5*s*cos(q1+q2)*(q1dd + q2dd);
Gddy = -l*cos(q1)*q1d^2 -l*sin(q1)*q1dd +0.5*s*sin(q1)*q1dd +0.5*s*cos(q1)*q1^2 -0.5*s*sin(q1+q2)*(q1dd+q2dd) -0.5*s*cos(q1+q2)*(q1d+q2d)^2;

Gdd = [Gddx ; Gddy];

F = 2*m*Gdd - 2*m*grav





end

