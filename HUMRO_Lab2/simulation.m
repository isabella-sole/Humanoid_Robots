function z_total = simulation(steps, X_star, time, state)

% Declaration of known variables 

l = 0.8;


%Initialize plot shift

xe = [0,0];


%Initialize total motion vector as an empty vector that will be filled at each iteration

z_total = [];


% Iteration

for j = 1:steps
    
    % Motion animation 
    
    hold off;
    xe = plot_motion(time, state, xe);
    hold on;          
    
    
    % State before impact
    
    xd_minus = -l*cos(X_star(1))*X_star(2);
    yd_minus = -l*sin(X_star(1))*X_star(2);
    z_minus = [xd_minus; yd_minus; X_star(2); X_star(3)];

    
    % Velocity after impact
    % Use the function written in lab 1

    [A1, J_R] = function_impact( X_star(1), pi-2*X_star(1));

    Ab = [A1, -J_R';J_R,zeros(2,2)];
    Ac = [A1 ; zeros(2,4)];

    
    % State after impact
    
    z_plus = Ab^(-1)*Ac*z_minus;
    q1d_plus = z_plus(3);
    q2d_plus = z_plus(4);

    
    % Assign new values at the new state

    X_star(2) = q1d_plus;
    X_star(3) = q2d_plus;
    
    
    % Relabeling equation
    E = [1,1,0,0; 0,-1,0,0; 0,0,1,1; 0,0,0,-1];
    
    % Simulate next step
    z0 = [X_star(1), pi-2*X_star(1), X_star(2), X_star(3)];
    z0 = E*z0'-[pi;0;0;0];

    
    % Set again the desired options
    
    options = odeset('Events', @PEvents);

    
    % Use this function to obtain: time (t), value of the state at time t (z), endtime (te), value of the state at endtime te (ze)

    [time, state, te, ze] = ode45(@SS_passif, [0:0.02:10], z0, options);
    X_star = [ze(1);ze(3);ze(4)];
    
    
    % Fill the z_total vector and update total motion
    
    z_total = [z_total; state];
end