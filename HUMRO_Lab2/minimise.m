%% Periodic motion function: it has to be minimised

function [following_X] = minimise(X)

% Declaration of known variables 

l = 0.8;

q1 = X(1);
q2 = pi-2*q1;
qp1 = X(2);
qp2 = X(3);


% State before impact 

xd = -l*cos(q1)*qp1;
yd = -l*sin(q1)*qp1;
z_minus = [xd;yd;qp1;qp2];


% Velocity after impact 
% Use the function written in lab 1

[A1, J_R] = function_impact(q1,q2);

Ab = [A1,-J_R';J_R,zeros(2,2)];
Ac = [A1 ; zeros(2,4)];


% State after impact

z_plus = Ab^(-1)*Ac*z_minus;
q1d_plus = z_plus(3);
q2d_plus = z_plus(4);


% Assign new positions and velocities values at the new state
ze(1) = q1;
ze(2) = q2;
ze(3) = q1d_plus;
ze(4) = q2d_plus;

% Relabeling equation

E = [1,1,0,0; 0,-1,0,0; 0,0,1,1; 0,0,0,-1];
z0 = E*ze'+[-pi;0;0;0];


% Set again the desired options

options = odeset('Events', @PEvents);


% Use this function to obtain: time (t), value of the state at time t (z), endtime (te), value of the state at endtime te (ze)

[t, z, te, ze] = ode45(@SS_passif, [0:0.02:10], z0, options);

following_X = [ze(1);ze(3);ze(4)];

end