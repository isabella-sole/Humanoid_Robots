%% HUMRO LAB 1: Humanoid and walking robots - Dynamic modeling of a compass
% Isabella-Sole Bisio

close all 
clear all
clc


%% My parameters

theta = 2*pi/180;

q1 = 1;
q2 = -0.5;

q1d = 0.1;
q2d = 0.2;

q1dd = 0.2;
q2dd = 0.3;


%% The dynamic model of the biped in single support

[A,H]=function_dyn(q1, q2, q1d, q2d, theta);


%% Reaction force

[F] = function_reactionforce(q1, q2, q1d, q2d, q1dd, q2dd);


%% The impact model

[A1, J_R] = function_impact(q1, q2);






