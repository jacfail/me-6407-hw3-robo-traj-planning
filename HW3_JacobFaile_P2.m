% Homework 3
% Jacob Faile
% 03/06/2024


%% Problem 2

% Select what solution you want to solve (1-4):
solution = 4;

% initialize the lengths of joints
la=1;
lb=0.2;
lc=1;

% initialize the external force and moment at the tip (world frame)
external_force=[0;0;-1];
external_moment=[0;1;0];

%% Choose what solution to solve:

if solution == 1
    % solution 1
    theta1=45/360*2*pi;
    theta2=-60/360*2*pi;
    theta3=120/360*2*pi;
elseif solution == 2 
    %solution 2 (same end point as 1)
    theta1=45/360*2*pi;
    theta2=60/360*2*pi;
    theta3=-120/360*2*pi;
elseif solution == 3
    %solution 3
    theta1=90/360*2*pi;
    theta2=45/360*2*pi;
    theta3=90/360*2*pi;
elseif solution == 4
    %solution 4
    theta1=180/360*2*pi;
    theta2=-30/360*2*pi;
    theta3=-60/360*2*pi;
end

%% Run calculations for finding the joint torques

% Redefined homogeneous transformation as functions of joint angles
T01=[    cos(theta1)     -sin(theta1)     0     0;
     sin(theta1)     cos(theta1)     0     0;
     0     0     1     0;
     0     0     0     1;
     ];

T12=[   cos(theta2)   -sin(theta2) 0  0;
         0         0    1         0;
    -sin(theta2)   -cos(theta2)         0         0;
         0         0         0    1;
         ];

T23=[     cos(theta3)   -sin(theta3)         0    la;
    sin(theta3)    cos(theta3)         0         0;
         0         0    1         0;
         0         0         0    1;
         ];

T34 = [1 0 0 lc;
    0 1 0 0; 
    0 0 1 lb;
    0 0 0 1];

T02 = T01*T12;
T03 = T01*T12*T23;
T04 = T01*T12*T23*T34;

% Find the end-point location
% Endpoint wrt Frame 3
r3=[lc;0;lb;1];

% find endpoint location wrt world frame
r0=T03*r3;


% Extract joint locations and draw manipulator
%Oo-O3-Om-O4
%Om is between O3 and O4
rm=T03*[0;0;lb;1];

% find all the joint locations wrt world frame
endpoint = [r0(1); r0(2); r0(3)];
j0 = [0; 0; 0];
j1 = [T03(1,4); T03(2,4); T03(3,4)];
j2 = [rm(1); rm(2); rm(3)];

% Calculate vectors from joints to endpoint in world frame
p0 = T04(1:3,4) - T01(1:3,4);
p1 = T04(1:3,4) - T02(1:3,4);
p2 = T04(1:3,4) - T03(1:3,4);

% Calculate the orientation of Z from the transformation matrices
z0 = T01(1:3,3);
z1 = T02(1:3,3);
z2 = T03(1:3,3);

% Take the cross product to find the top row of the Jacobian
v0 = cross(z0, p0);
v1 = cross(z1, p1);
v2 = cross(z2, p2);

% Fill in the Jacobian with the calculated values
J = [v0, v1, v2; z0, z1, z2];

% External wrench in the end-effector frame
external_wrench = [-external_force; -external_moment];

% Calculate joint torques (inverse dynamics)
joint_torques = J' * external_wrench;

% Set the joint torque tau values for the SimScape model
tau1=joint_torques(1);
tau2=joint_torques(2);
tau3=joint_torques(3);


% disp('Jacobian for Solution 1:')
% disp(J)
% 
% disp('Joint Torques for Solution 1:')
% disp(joint_torques)


% % Plot the manipulator (if desired)
% figure(1)
% hold on
% plot3([0.1 -0.1 -0.1 0.1 0.1],[0.1 0.1 -0.1 -0.1 0.1],[0 0 0 0 0])  %base square
% plot3([0 T03(1,4) rm(1) r0(1)],[0 T03(2,4) rm(2) r0(2)],[0 T03(3,4) rm(3) r0(3)])
% plot3(r0(1),r0(2),r0(3),'o');
% axis equal
% 
% grid
% hold off
% view(-30,30)
