clear all, close all, clc

%% Given the following state space representation
%  x' = [-b/m]x + [1/m]u
%  y = Cx

nx = 2; % 2 states of input x: velocity and acceleration (v and v')
ny = 1; % 1 state of input y: velocity

m = 5 % mass of the car 

a = [0 1; (-m/(250*m)) 1] % A matrix, use math and brain power to deduce, good luck =)))
b = [0; 1/m] % same as A matrix 
c = [1 1] % C matrix
d = 0; % for simplicity, lets say disturbance in the output is 0; meaning, y = Cx

H = ss(a,b,c,d) % build the car's dynamic 

%G = tf(H) % get transfer function... Just for fun, it doesnt do anything in this method

p = size(c,1)
[n,m] = size(b)

%% Q and R here are to build the cost function J
Q = [1, 0; 0, 10]; 
R = eye(m)

Kr = lqr(a,b,Q,R) % find the best gain K for the regulator (tuning technique)

%% Define Noise and Disturbance covariance
Bnoise = eye(n)
W = eye(n)
V = 0.01*eye(m)

%% Estimated state space and Kalman filter construction
Estss = ss(a, [b Bnoise], c, zeros(1,3))

[Kess, Ke] = kalman (Estss,W,V) 
