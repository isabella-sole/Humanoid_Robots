function [XEN] = plot_motion(T, Z, XE)

% Set the length of the leg

l = 0.8;


% Iteration for all the time instants

for i = 1: length(T)

base = XE; % Set the value of the base

hip = [-l*sin(Z(i,1))+XE(1), l*cos(Z(i,1))]; % Set the value of the hip

tip = [-l*sin(Z(i,1))-l*sin(Z(i,1)+Z(i,2))+XE(1), l*cos(Z(i,1))+l*cos(Z(i,1)+Z(i,2))]; % Set the value of the tip

points = [base; hip; tip];

hold off;
plot(points(:,1),points(:,2), '-b', 'LineWidth', 3);
axis([-1 20 0 2]);
hold on;

drawnow;

end

XEN = tip;

end