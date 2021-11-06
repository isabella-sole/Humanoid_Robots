function [J]=resol(Jsolcons0)

global m;
global g;
global l;
global theta;
global gt;


%% COST COMPUTATION

% Extract data to be optimzed
q1_fin = Jsolcons0(1);
q2_fin = pi-2*q1_fin;
q1d_fin = Jsolcons0(2);
q2d_fin = Jsolcons0(3);
q1_int = Jsolcons0(4);
q2_int = Jsolcons0(5);
T = Jsolcons0(6);

% Build impact vector
ze = [q1_fin, q2_fin, q1d_fin, q2d_fin];

% Compute hip velocities
xd = -l*cos(ze(1))*ze(3);
yd = -l*sin(ze(1))*ze(3);

zminus = [xd;yd;ze(3);ze(4)];

% Compute joints velocity after impact 

[A1, JR] = function_impact( ze(1), ze(2));

Ab = [A1, -JR';JR,zeros(2,2)];
Ac = [A1 ; zeros(2,4)];

zplus =  pinv(Ab)*Ac*zminus;

q1dplus = zplus(3);
q2dplus = zplus(4);

ze(3) = q1dplus;
ze(4) = q2dplus;

% Extract impact forces
Ix = zplus(5);
Iy = zplus(6);

% Perform relabeling
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

% Compute the parameters for the polynomial trajectory
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

% Compute the polyomials
delta = 100;
t = linspace(0,T,delta)';

fi1 = a01 + a11 * t + a21 * t.^2 + a31 * t.^3 + a41 * t.^4;
fi1d = a11 + 2* a21 * t + 3 * a31 * t.^2 + 4 * a41 * t.^3;
fi1dd = 2 * a21 + 6 * a31 * t + 12 * a41 * t.^2;

fi2 = a02 + a12 * t + a22 * t.^2 + a32 * t.^3 + a42 * t.^4;
fi2d = a12 + 2* a22 * t + 3 * a32 * t.^2 + 4 * a42 * t.^3;
fi2dd = 2 * a22 + 6 * a32 * t + 12 * a42 * t.^2;

% Compute distance 
d = 2*l*sin(q1_fin);

% Initializations
J = 0;

% LOOP for the integration of the cost to build J
for i = 1: delta

% Compute dynamic model
[A,H,Q] = function_dyn(fi1(i),fi2(i),fi1d(i),fi2d(i),theta);

qdd = [fi1dd(i);fi2dd(i)];

% Invert DYN in order to get input torques
tau(:,i) = (A*qdd + H + Q);

% Compute reactions forces
R = function_reactionforce(fi1(i),fi2(i),fi1d(i),fi2d(i),fi1dd(i),fi2dd(i));
Rx(i) = R(1);
Ry(i) = R(2);

% Evaluate J
J = J + (1/(2*m*g*d))*(tau(1,i)^2+tau(2,i)^2)*(T/delta);
end

%% CONSTRAINTS  DEFINITION

 gt = [abs(Rx)-0.7*abs(Ry); -Ry; -d+0.2*ones(1,delta); -Iy*ones(1,delta); abs(Ix)*ones(1,delta)-0.7*abs(Iy)*ones(1,delta); abs(fi1d)-3*ones(1,delta); abs(fi2d)-3*ones(1,delta); 
     abs(tau(1))-50*ones(1,delta); abs(tau(2))-50*ones(1,delta)];

end

