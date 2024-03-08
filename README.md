# me-6407-hw3-robo-traj-planning

## Overview
This code is all done in MATLAB. It covers topics like transformation matrices, inverse and forward kinematics, trajectory planning, and external wrenches.

There are two scripts, one helper function, and two simulink models. A requirement is that you use MATLAB and SIMULINK with Simscape Multibody.


## Part 1
### Description

Code: `HW3_JacobFaile_P1.m`

In problem 1-1, the code was done in MATLAB. The parameters of the robot, the center and radius of the circle, the delta, the theta of the end effector start and end, and the time range were all initialized. I then found the theta range for generating points (from `0 -> pi/2` for the bottom right quadrant), then generated x and y coordinates for the circle from those theta values.

After that it was pretty straight forward to run through the provided `fourpolytraj.m` code. The velocity at `2*delta` and `tf-2*delta` were both established. Acceleration starts at 0, ramps up the velocity to this calculated amount, stays constant, then negative acceleration is applied until velocity decreases back to 0 at our end point.

The velocity of the theta end effector I calculated was `-1.0472 m/sec` (I am assuming those are the units for the lengths given). Using this new velocity of theta, I was able to find the position at `2 * delta time` and `tf - 2 * delta time`. I then iterated through the timesteps and found the theta of the end effector at the 3 regions that I separated everything into, with the constant velocity in the center.

Now that I have the theta of the end effector, I was able to find the position of the end effector as a function of theta and radius, with radius being a constant. The equation for that was `xed = center_x + radius *cos(theta_endeff)` and `yed = center_y + radius * sin(theta_endeff)`.

All I had to do after that was perform the inverse kinematics of those end effector positions to find all the joint angles for all those positions.

I then used these values and inputted them into the SimScape model to run the simulation on the robot arm. 

 
Problem 1-2 is very similar. I was able to extract the Vx and Vy dat from the simscape model and plot it. 

The resulting values are the same. The numerically differentiated joint angles and end-point velocities calculated from the Jacobian matches the SimScape models end-point velocity data.


`HW03_planar3Rrobot.slx` is the simulink model for `HW3_JacobFaile_P1.m`.


## Part 2

### Description

Code: `HW3_JacobFaile_P2.m`

For this problem, I needed to determine the joint torques to resist the external wrench provided to make sure the robot does not move. 

The way to make this happen is to obtain the Jacobian matrix for each of the postures and compute the required torques. I was able to check my work by running a SimScape simulator. If the manipulator moved at all, then I knew that my implementation was not correct.

The way I was able to get these results was by first initialize the lengths of links, external force/moment, and the joint angles. I then defined the homogenous transformation matrices as a function of joint angles. 

After that, I was able to use those transformations to find the joint locations in the world frame, including the end point of the manipulator. I then calculated the P vectors from the joints to the endpoints in the world frame and the orientation of the joint Z values from the transformation matrices.

The joint Z orientations were the lower three rows of my new Jacobian. When I took the cross product of the P vectors with the joint Z orientations, I was able to fill in the Jacobian matrix. 

I negated the external force and moment (because we want to resist it), took the transpose of the Jacobian matrix and used inverse dynamics to calculate the joint torques. These values (tau1, tau2, tau3) were inputted into the Simscape simulator to test. 

All three solutions successfully kept the manipulator stationary and were able to resist the external wrench provided. 


`HW03_simplified_PUMA_statics_simscape.slx` is the simulink model for `HW3_JacobFaile_P2.m`.
