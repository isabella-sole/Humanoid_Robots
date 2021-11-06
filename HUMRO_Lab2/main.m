%% HUMRO LAB 2: Humanoid and walking robots – Passive walking of a compass robot
% Isabella-Sole Bisio

close all 
clear 
clc


%% Declaration of variables 

% global variables  

global I;
global s;

I = 0.1; % [kg*m^2]
s = 0.5; % [0.5m]


% local variables

l = 0.8; % [m]
 

%% 1) Simulation of the passive gait
%% A Simulation of one single support phase


% Initial conditions

q1 = 0.1860; 
q2 = 2.7696;
qd1 = -1.4281;
qd2 = 0.3377;


% Set the desired options

options = odeset('Events', @PEvents);


% Set the initial state for the integration

z0=[q1;q2;qd1;qd2];


% Use this function to obtain: time (t), value of the state at time t (z), endtime (te), value of the state at endtime te (ze)

[t, z, te, ze] = ode45(@SS_passif, [0:0.02:10], z0, options);


% Passive walking animation

figure;
xe = [0,0];
xe = plot_motion(t, z, xe);


%% B Simulation of several half steps

% State before impact

xd_minus = -l*cos(ze(1))*ze(3);
yd_minus = -l*sin(ze(1))*ze(3);
z_minus = [xd_minus; yd_minus; ze(3); ze(4)];


% Velocity after impact 
% Use the function written in lab 1

[A1, J_R] = function_impact( ze(1), ze(2));

Ab = [A1, -J_R' ; J_R, zeros(2,2)];
Ac = [A1 ; zeros(2,4)];


% State after impact

z_plus = Ab^(-1)*Ac*z_minus;
q1d_plus = z_plus(3);
q2d_plus = z_plus(4);


% Assign new velocities value at the new state

ze(3) = q1d_plus;
ze(4) = q2d_plus;


% Relabeling equation

E = [1,1,0,0; 0,-1,0,0; 0,0,1,1; 0,0,0,-1];


% Simulate next step

z0 = E*ze'-[pi;0;0;0];


% Set again the desired options

options = odeset('Events', @PEvents);


% Use this function to obtain: time (t), value of the state at time t (z), endtime (te), value of the state at endtime te (ze)

[t, z, te, ze] = ode45(@SS_passif, [0:0.02:10], z0, options);


% Animation of the passive walking

xe = plot_motion(t,z,xe);