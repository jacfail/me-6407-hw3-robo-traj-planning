%% Planar 3R robot forward and inverse kinematics

a1=0.2;
a2=0.15;
a3=0.12;
d1=0.1;
d2=0.1;
d3=0.1;

%% forward kinematics

theta1=20/360*2*pi;
theta2=40/360*2*pi;
theta3=15/360*2*pi;

S1=sin(theta1);
S12=sin(theta1+theta2);
S2=sin(theta2);
S123=sin(theta1+theta2+theta3);
S23=sin(theta2+theta3);
S3=sin(theta3);
C1=cos(theta1);
C2=cos(theta2);
C3=cos(theta3);
C12=cos(theta1+theta2);
C23=cos(theta2+theta3);
C3=cos(theta3);
C123=cos(theta1+theta2+theta3);

%from manipulator geometries
x4pos=a1*C1+a2*C12+a3*C123
y4pos=a1*S1+a2*S12+a3*S123
theta1+theta2+theta3

x3pos=a1*C1+a2*C12; %inverse kinematics solution starts from here for simplicity
y3pos=a1*S1+a2*S12;


figure(1)
hold on
plot([0 a1*C1 a1*C1+a2*C12 a1*C1+a2*C12+a3*C123 x4pos],[0 a1*S1 a1*S1+S12*a2 a1*S1+S12*a2+a3*S123 y4pos])
plot(x4pos,y4pos,'o');
axis equal
grid

%Homogeneous transformation
T01=[C1 -S1 0 0;S1 C1 0 0;0 0 1 d1; 0 0 0 1];
T12=[C2 -S2 0 a1;S2 C2 0 0;0 0 1 d2; 0 0 0 1];
T23=[C3 -S3 0 a2;S3 C3 0 0;0 0 1 d3; 0 0 0 1];
T34=[1 0 0 a3;0 1 0 0;0 0 1 0; 0 0 0 1];

T04=T01*T12*T23*T34

%% inverse kinematics

%see if we can reproduce theta1, 2 and 3

p13=sqrt((a1*C1+a2*C12)^2+(a1*S1+a2*S12)^2);

tmp=inv([a1+a2*C2 -a2*S2;a2*S2 a1+a2*C2])*[x3pos;y3pos];

theta1new=atan2(tmp(2),tmp(1))

theta2new=atan2(sqrt(1-((p13^2-a1^2-a2^2)/(2*a1*a2))^2),(p13^2-a1^2-a2^2)/(2*a1*a2))



%% numerical solution
syms theta1sol theta2sol
eq1=a1*cos(theta1sol)+a2*cos(theta1sol+theta2sol)==x3pos;  %x3pos
eq2=a1*sin(theta1sol)+a2*sin(theta1sol+theta2sol)==y3pos;  %y3pos

S=vpasolve([eq1 eq2],[theta1sol theta2sol],[0 0]);

S.theta1sol
S.theta2sol