startup_rtb
clear; clc;

%% Homogeneous transformation in 2D in Matlab
% 2D translation by 1 unit on x and 2 units on y
transl2(1,2)

% 2D Rotation matrix rotating 30 deg
rot2(30, 'deg')

% 2D Rotation matrix rotating 0 rad for homogeneous transformation
trot2(30, 'deg')

% Combining the translation and rotation
transl2(1,2) * trot2(30, 'deg')


%% Alternative
% One can define the same transformation in SE as
SE2(1,2,30,'deg')

%% Complete example
% Set the axis
axis([0,5,0,5]); axis square; hold on
T1 = SE2(1,2,30,'deg');
T2 = SE2(1,2, 0,'deg');
T3 = T1 * T2;
T4 = T2 * T1;
% Plot T1 as a frame named 1 in blue
trplot2(T1, 'frame', '1', 'b')
% Plot T2 as a frame named 2 in red
trplot2(T2, 'frame', '2', 'r')
% Plot T3 as a frame named 3 in green
trplot2(T3, 'frame', '3', 'g')
% Plot T4 as a frame named 3 in green
trplot2(T4, 'frame', '4', 'c')
% Define a point in the world frame
P = [3,2]';
plot_point(P,'*');
% Compute the coordinates of P in frame 1
P1 = inv(T1.T) * [P; 1];