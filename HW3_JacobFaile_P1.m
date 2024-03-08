% Homework 3
% Jacob Faile
% 03/06/2024


%% Problem 1-1

timestep = 0.0001;

%desired end point location and orientation
t0=0;
tf=2;

% parameters of the robot
a1=0.2;
a2=0.15;
a3=0.12;
d1=0.1;
d2=0.1;
d3=0.1;

% Define the center of the circle and the radius
center_x = 0.1;­­­
center_y = 0.25; 
radius = 0.25;

tt=t0:0.001:tf;

theta_initial = 0;
theta_final = -pi/2;

delta=0.25;

% Define the angle range for generating points (from 0 to pi/2 for the bottom-right quadrant)
theta_endeff = linspace(theta_initial, theta_final, numel(tt));

% Generate x and y coordinates for the circle
x = center_x + radius * cos(theta_endeff);
y = center_y + radius * sin(theta_endeff);



% velocity at 2*delta and tf=2*delta
dtheta_delta=(theta_final-theta_initial)/(tf-t0-2*delta);

% theta position at 2*delta
theta_2delta=theta_initial + dtheta_delta*delta;

% theta position at tf-2*delta
theta_tfm2delta=theta_final - dtheta_delta*delta;

% run fourpolytraj on the theta of the end effector
theta_endeff = zeros(1,length(tt));

for ii=1:length(tt)
    
    if tt(ii)<2*delta %first 4-th poly
        theta_endeff(ii)= fourpolytraj(tt(ii), theta_initial, theta_2delta, t0, 2*delta, 0, dtheta_delta);
    elseif tt(ii)>tf-2*delta %last 4-th poly
        theta_endeff(ii)= fourpolytraj(tt(ii), theta_tfm2delta, theta_final, tf-2*delta, tf, dtheta_delta, 0);
    else 
        theta_endeff(ii)= theta_initial + dtheta_delta*(tt(ii)-delta);
        %constant velocity
    end
end

% find the position of the end effector as a function of the theta
xed = center_x + radius*cos(theta_endeff);
yed = center_y + radius*sin(theta_endeff);


% do the inverse kinematics of the sys to get joint angles for all pos
for ii=1:length(tt)
    x3posd=xed(ii)-a3*cos(theta_endeff(ii));
    y3posd=yed(ii)-a3*sin(theta_endeff(ii));

    %determine O3 location for inverse kinematics
    p13=sqrt(x3posd^2+y3posd^2);
    C2inv=(p13^2-a1^2-a2^2)/2/a1/a2;
    C2=C2inv;

    % by taking the negative calculated we can avoid hitting the wall
    theta2invp=atan2(sqrt(abs(1-((p13^2-a1^2-a2^2)/(2*a1*a2))^2)),(p13^2-a1^2-a2^2)/(2*a1*a2));
    theta2invm=-atan2(sqrt(abs(1-((p13^2-a1^2-a2^2)/(2*a1*a2))^2)),(p13^2-a1^2-a2^2)/(2*a1*a2));

    % calculate all the joint angles
    theta2inv(ii)=theta2invm;
    S2=sin(theta2inv(ii));

    tmp=inv([a1+a2*C2 -a2*S2;a2*S2 a1+a2*C2])*[x3posd;y3posd];
    theta1inv(ii)=atan2(tmp(2),tmp(1));

    theta3inv(ii)=theta_endeff(ii)-theta1inv(ii)-theta2inv(ii);
end

% plot the theta of end effector against time
figure(1)
plot(tt, theta_endeff, 'b')
grid on
xlabel('Time (tt)')
ylabel('Theta_EndEff')
title('Theta of End Effector vs Time')
legend('Theta of EndEff')

% plot the joint angles against time
figure(2)
plot(tt, theta1inv, 'b', tt, theta2inv, 'r', tt, theta3inv, "MarkerFaceColor", [0.9290 0.6940 0.1250])
grid on
xlabel('Time (tt)')
ylabel('Joint angles')
title('Joint angles vs Time')
legend('Theta1', 'Theta2', 'Theta3')

% generate time series data for simulink/simscape
theta1dsim=[tt',theta1inv'];
theta2dsim=[tt',theta2inv'];
theta3dsim=[tt',theta3inv'];



%% Problem 1-2

% extract the data from the simscape model
sim_Vx = planar3Rmotiondata.Data(:,5);
sim_Vy = planar3Rmotiondata.Data(:,6);
sim_tt = planar3Rmotiondata.Time;

% first plot the simulated vx and vy from simscape
figure(3)
hold on
plot(sim_tt, sim_Vx, "color", "b");
plot(sim_tt, sim_Vy, "color", [0.9290 0.6940 0.1250]);

xlabel("Time (sec)")
ylabel("Velocity")
title("SimScape Vx and Vy")
grid on






% find the dtheta matrix by taking the deriv of all theta values
dtheta = [(diff(theta1dsim(:,2))./timestep), (diff(theta2dsim(:,2))./timestep), (diff(theta3dsim(:,2))./timestep); 
    0, 0, 0];

% setup the jacobian parameters for easy access
S1 = sin(theta1dsim(:,2));
S12 = sin(theta1dsim(:,2) + theta2dsim(:,2));
S123 = sin(theta1dsim(:,2) + theta2dsim(:,2) + theta3dsim(:,2));

C1 = cos(theta1dsim(:,2));
C12 = cos(theta1dsim(:,2) + theta2dsim(:,2));
C123 = cos(theta1dsim(:,2) + theta2dsim(:,2) + theta3dsim(:,2));

% create the row of x and y for the jacobian
J_x = [-a1*S1-a2*S12-a3*S123, -a2*S12-a3*S123, -a3*S123];
J_y = [a1*C1+a2*C12+a3*C123, a2*C12+a3*C123, a3*C123];

% find the Vx and Vy values by taking the dot product
Vx = dot(dtheta, J_x, 2);
Vy = dot(dtheta, J_y, 2);

figure(4)
hold on
plot(tt, Vx, "color", "b");
plot(tt, Vy, "color", [0.9290 0.6940 0.1250]);

xlabel("Time (sec)")
ylabel("Velocity")
title("Reproduced Vx and Vy")
grid on
