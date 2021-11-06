%% HUMRO LAB 2: Humanoid and walking robots – Passive walking of a compass robot
% Isabella-Sole Bisio

close all 
clear 
clc


%% CASE 1 - Periodic motion 2
%% Declaration of known variables 

% global variables  

global I;
global s;

I = 0.08; % [kg*m^2]
s = 0.45; % [0.5m]


% % Initial conditions 

q1 = 0.1860;
q2 = 2.7696;
qd1 = -1.4281;
qd2 = 0.3377;


%% Initialize the environment for Periodic motion 2 

figure;
xe = [0,0]; % Starting point


% Set the desired options

options = odeset('Events', @PEvents);


% Set the initial state for the integration

z0=[q1;q2;qd1;qd2];


% Use this function to obtain: time (t), value of the state at time t (z), endtime (te), value of the state at endtime te (ze)

[t, z, te, ze] = ode45(@SS_passif, [0:0.02:10], z0, options);

X0 = [ze(1);ze(3);ze(4)]; % Assign X0 values using the ze vector just found


% X for periodic motions

options = optimset('display','iter','MaxFunEvals',1000,'TolX', 1.0e-010,'TolFun', 1.0e-006);
Xstar = fsolve('periodic', X0, options); % Value for periodic motions


%%  Periodic motion

% z_total = simulation(30, Xstar, t, z); % simulate 30 steps      DOES NOT FOLLOW DOWN
 z_total = simulation(50, Xstar, t, z); % simulate 50 steps      DOES NOT FOLLOW DOWN
% z_total = simulation(200, Xstar, t, z); % simulate 200 steps      DOES NOT FOLLOW DOWN


% Plot resulting phase plane

Xstar = [Xstar(1), pi-2*Xstar(1), Xstar(2), Xstar(3)];


% Iteration

for i = 1:size(z_total,1)
    
    
    % Wrap angle in radians to [0 2*pi]
    
    z_total(i,1) = wrapTo2Pi(z_total(i,1));
    z_total(i,2) = wrapTo2Pi(z_total(i,2));
    
    
end

Xstar(1) = wrapTo2Pi(Xstar(1));
Xstar(2) = wrapTo2Pi(Xstar(2));


%% Plot the Displacement-Velocity correlation

% Displacement-Velocity correlation for q1, q1d
figure;
hold on;
plot(z_total(:,1),z_total(:,3));
plot(Xstar(1), Xstar(3), '*r');
title('Phase plane in q1,q1d - Periodic motion 2');
xlabel('X: Displacement');
ylabel('Y: Velocity');
hold off;

figure;
hold on;
plot(z_total(:,2),z_total(:,4));
plot(Xstar(2), Xstar(4), '*r');
title('Phase plane in q2,q2d - Periodic motion 2');
xlabel('X: Displacement');
ylabel('Y: Velocity');
hold off;


%% Analisys of the stability

 stability = stability_check(Xstar);
 
 
 % Based on the reutrn value of the stability_check function we can have two cases:
 
 if stability == 0
     
     disp('The periodic motion is UNSTABLE for Periodic motion 2');
     
 elseif stability == 1
     
     disp('The periodic motion is STABLE for Periodic motion 2');
        
 end

