function [VALUE, ISTERMINAL, DIRECTION] = PEvents(T,Z)

l = 0.8; % [m] length of the leg


%% State values extracted from vector Z

q1 = Z(1); % Position of q1
q2 = Z(2); % Position of q2
 

%% Position of the tip 

X = -l*sin(q1)-l*sin(q1+q2); % X coordinate

Y = l*cos(q1)+l*cos(q1+q2); % Y coordinate


%% Set DIRECTION and ISTERMINAL
% Fixed values as suggested in the text of the assignment

DIRECTION = 0;  
ISTERMINAL = 1;


%% Set VALUE
% Changing value: this if is performed in order to avoid that the step stop when the top leg tips coincide in (0,0)

if X > 0.1
    
    VALUE = Y;
    
else
    
    VALUE = 1;
    
end


end