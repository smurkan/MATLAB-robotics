startup_rtb
clear; clc;

%% Define rotation
% 2D Rotation matrix rotating 0 rad
rot2(0)

% 2D Rotation matrix rotating 0.2 rad
 rot2(0.2)
 
% 2D Rotation matrix rotating 30 degrees and store it in R
 R = rot2(30, 'deg')
 
 
%% Rotation matrix properties
% Verify orthogonality of the columns
c1 = R(:,1); c2 = R(:,2);
dot(c1,c2)

% Verify determinant equal to 1
det(R)

% Verify that the inverse is equal to the transpose
inv(R)

R'

inv(R)-R'

%% Visualize the rotation matrix as a rotation of the coordinate frame
trplot2(R)
axis equal
