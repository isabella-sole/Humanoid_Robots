function [ZDOT] = SS_passif(T, Z)
%% State values extracted from Z vector

q1 = Z(1); % Position of q1
q2 = Z(2); % Position of q2
q1d = Z(3); % Velocity of q1
q2d = Z(4); % Velocity of q2


%% Known quantities

theta = 3*pi/180; % [rad]


%% Dyamic model of the robot

[A, H, Q] = function_dyn(q1,q2,q1d,q2d,theta); % Function built in the first Lab
% Q = Q'; % Transpose of Q to match dimensions


%% Compute the accelerations
% Extract qdd using the previous line evaluation

qdd = A^(-1)*(-H-Q);


%% Define the derivative of the state

ZDOT(1) = q1d;
ZDOT(2) = q2d;
ZDOT(3) = qdd(1);
ZDOT(4) = qdd(2);
     
ZDOT = ZDOT';


end