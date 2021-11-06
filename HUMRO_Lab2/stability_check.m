function [stab] = stability_check(X)

% Firstly assume STABLE motion

stab = 1;


% Initialise errors

velocity_error = 0.0005;
position_error = 0.00005;


% Jacobian matrix J

J1_1 = minimise(X + [1;0;0]*position_error);
J1_2 = minimise(X - [1;0;0]*position_error);

J2_1 = minimise(X + [0;1;0]*velocity_error);
J2_2 = minimise(X - [0;1;0]*velocity_error);

J3_1 = minimise(X + [0;0;1]*velocity_error);
J3_2 = minimise(X - [0;0;1]*velocity_error);


J1 = (J1_1 - J1_2)/(2*position_error);
J2 = (J2_1 - J2_2)/(2*velocity_error);
J3 = (J3_1 - J3_2)/(2*velocity_error);

J = [J1,J2,J3];


% Jacobian matrix eigenvalues

Eig = eig(J);


% Check if the norm of each eigenvalue is < 1
% Iteration


for i= 1:length(Eig)

    if norm(Eig(i)) >= 1
        
       stab = 0; % UNSTABLE motion
       
    end
        
end



end