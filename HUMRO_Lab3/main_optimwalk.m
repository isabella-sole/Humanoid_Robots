%% HUMRO LAB 3: Humanoid and walking robots – Assignment Optimal Walking
% Isabella-Sole Bisio

close all 
clear 
clc


%% Declaration of variables 
% Define a global variables

global m;
global I;
global g;
global l;
global s;
global theta;


% I have chosen the parameters using CASE 2 of the previous lab, since it was the more stable motion

m = 2; % [kg]
I = 0.08; % [kg m^2]
g = 9.81; % [m s^(-2)]
l = 0.8; % [m]
s = 0.45; % [m]
theta = 0;




%% Write polynomial functions
% Define the structure of the polynomial function defining a period and a trajectory 

T = 1;
q1_fin = deg2rad(-10);
% q2_fin = pi-2*q1_fin;
q1d_fin = 0;
q2d_fin = 0;
q1_int = 0;
q2_int = -pi;


% Initialize optimization vector

Jsolcons0 = [q1_fin, q1d_fin, q2d_fin, q1_int, q2_int, T];


% Perform optimization
lb = [];
ub = [];
options1 = optimset('Display','iter','MaxFunEvals',100000,'MaxIter',10000,'LargeScale','off');
[Jsolcons,Fval,EXITFLAG, OUTPUT] = fmincon('resol',Jsolcons0,[],[],[],[],lb,ub,'constraint',options1);
% disp(OUTPUT.message);

% Reinitialize vector with optimized data
q1_fin = Jsolcons(1);
% q2_fin = (1/2)*(pi-q2_fin);
q2_fin = pi-2*q1_fin;
q2d_fin = Jsolcons(3);
q1d_fin = Jsolcons(2);
q1_int = Jsolcons(4);
q2_int = Jsolcons(5);
T = Jsolcons(6);

ze = [q1_fin, q2_fin, q1d_fin, q2d_fin];

% Compute hip velocities
xd = -l*cos(ze(1))*ze(3);
yd = -l*sin(ze(1))*ze(3);

zminus = [xd;yd;ze(3);ze(4)];

% Joint velocity after impact 

[A1, JR] = function_impact( ze(1), ze(2));

Ab = [A1, -JR';JR,zeros(2,2)];
Ac = [A1 ; zeros(2,4)];

zplus = pinv(Ab)*Ac*zminus;

q1di = zplus(3);
q2di = zplus(4);

ze(3) = q1di;
ze(4) = q2di;

% Compute Impact forces
Ix = zplus(5);
Iy = zplus(6);

% Relabeling
E = [1,1,0,0;
    0,-1,0,0;
    0,0,1,1;
    0,0,0,-1];

z0 = E*ze'-[pi;0;0;0];

% Update initial positions and velocities
q1i = z0(1);
q2i = z0(2);
q1di = z0(3);
q2di = z0(4);

% Compute the coefficients for the polynomial trajectory
fi1 = [q1i; q1di;q1_int; q1_fin; q1d_fin];
fi2 = [q2i; q2di;q2_int; q2_fin; q2d_fin];


t_mat = [     0,       0,       0,     0, 1;
              0,       0,       0,     1, 0;
        (T/2)^4, (T/2)^3, (T/2)^2, (T/2), 1;
            T^4,     T^3,     T^2,     T, 1;
          4*T^3,   3*T^2,     2*T,     1, 0];

Ac = pinv(t_mat)*fi1;
Ac2 = pinv(t_mat)*fi2;
a01 = Ac(5);
a02 = Ac2(5);
a11 = Ac(4);
a12 = Ac2(4);
a21 = Ac(3); 
a22 = Ac2(3);
a31 = Ac(2);
a32 = Ac2(2);
a41 = Ac(1);
a42 = Ac2(1);

% Compute the actual polynomial values for the two joints
delta = 100;
t = linspace(0,T,delta)';

q1 = a01 + a11 * t + a21 * t.^2 + a31 * t.^3 + a41 * t.^4;
q2 = a02 + a12 * t + a22 * t.^2 + a32 * t.^3 + a42 * t.^4;

qd1 = a11 + 2* a21 * t + 3 * a31 * t.^2 + 4 * a41 * t.^3;
qd2 = a12 + 2 * a22 * t + 3 * a32 * t.^2 + 4 * a42 * t.^3;

z = [q1, q2];


%% Plot
% Give the profiles of qi, qdoti, Rx, Ry, and taui

% Plot the motion
origin = [0,0];
plot_motion(t, z, origin);
title('Motion of the walking robot');
xlabel('time');


% Plot q1 and q2
figure;
subplot(1,2,1);
plot(q1,'k');
title('q1 trend');
xlabel('time');
ylabel('amplitude');
subplot(1,2,2);
plot(q2,'k');
title('q2 trend');
xlabel('time');
ylabel('amplitude');


% Plot qd1 and qd2
figure;
subplot(1,2,1);
plot(qd1, 'k');
title('qd1 trend');
xlabel('time');
ylabel('amplitude');
subplot(1,2,2);
plot(qd2, 'k');
title('qd2 trend');
xlabel('time');
ylabel('amplitude');


% Plot Torque

% Second derivatives
qdd1 = 2 * a21 + 6 * a31 * t + 12 * a41 * t.^2;
qdd2 = 2 * a22 + 6 * a32 * t + 12 * a42 * t.^2;

for i = 1 : delta
[A,H,Q] = function_dyn(q1(i), q2(i), qd1(i), qd2(i), theta);
qdd = [qdd1(i) ; qdd2(i)];

% Model
tau(:,i) = A*qdd + H + Q;
figure(4)
plot(tau)

end



