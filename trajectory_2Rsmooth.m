%% Planar 2R trajectory planning 
% joint level: 4th poly -constact velcoty - 4th poly
% end point:  4th poly -constact velcoty - 4th poly
% this scirpt calls matlab function "fourpolytraj.m"

a1=0.2;
a2=0.15;

%% joint trajectory parameters
% joint control
t0=0;
tf=2;

theta1t0=0/360*2*pi;
theta1tf=90/360*2*pi;
theta2t0=30/360*2*pi;
theta2tf=90/360*2*pi;

delta=0.25;

% velocity at 2*delta and tf=2*delta
dtheta1delta=(theta1tf-theta1t0)/(tf-t0-2*delta)
dtheta2delta=(theta2tf-theta2t0)/(tf-t0-2*delta)

% position at 2*delta
theta1_2delta=theta1t0+dtheta1delta*delta;
theta2_2delta=theta2t0+dtheta2delta*delta;

% position at tf-2*delta
theta1_tfm2delta=theta1tf-dtheta1delta*delta;
theta2_tfm2delta=theta2tf-dtheta2delta*delta;

%plot trajectories
%call trajectory generation function
%y=fourpolytraj(t,x0,xf,t0,tf,dx0,dxf)
%boundary condition: (xe-x0)=tf/2*(dxf-dx0)

% test taking t0-tf and plot one 4th order poly 
% figure(1)
% tt=t0:0.05:tf;
% theta1d=fourpolytraj(tt,theta1t0,theta1tf,t0,tf,0,(theta1tf-theta1t0)/tf*2);
% plot(tt,theta1d)
% grid
% 
% figure(2)
% tt=t0:0.05:tf;
% theta1d=fourpolytraj(tt,theta1t0,theta1tf,t0,tf,(theta1tf-theta1t0)/tf*2,0);
% plot(tt,theta1d)
% grid

% generating 4th order poly for 0-2delta, constant velocity for 2deta-(tf-2delta)
% and 4th order for (tf-2delta) to tf 

tt=t0:0.05:tf;

for ii=1:length(tt),
    
if tt(ii)<2*delta, %first 4-th poly
   theta1d(ii)=fourpolytraj(tt(ii),theta1t0,theta1_2delta,t0,2*delta,0,dtheta1delta);
   theta2d(ii)=fourpolytraj(tt(ii),theta2t0,theta2_2delta,t0,2*delta,0,dtheta2delta);
elseif tt(ii)>tf-2*delta, %last 4-th poly
    theta1d(ii)=fourpolytraj(tt(ii),theta1_tfm2delta,theta1tf,tf-2*delta,tf,dtheta1delta,0);
    theta2d(ii)=fourpolytraj(tt(ii),theta2_tfm2delta,theta2tf,tf-2*delta,tf,dtheta2delta,0);
else theta1d(ii)=theta1t0+dtheta1delta*(tt(ii)-delta);
    theta2d(ii)=theta2t0+dtheta2delta*(tt(ii)-delta);
    %constant velocity
end
end

figure(1)
plot(tt,theta1d)
grid

figure(2)
plot(tt,theta2d)
grid

% generate time series data for simulink/simscape
% to see this result on SimScape, insert break
theta1dsim=[tt',theta1d']
theta2dsim=[tt',theta2d']



%% task-space trajectory parameters


%desired end point location and orientation
t0=0;
tf=2;

xt0=-0.2;
xtf=0.2;
yt0=0.2;
ytf=0.2;

delta=0.25;

% velocity at 2*delta and tf=2*delta
dxdelta=(xtf-xt0)/(tf-t0-2*delta)
dydelta=(ytf-yt0)/(tf-t0-2*delta)

% position at 2*delta
x_2delta=xt0+dxdelta*delta;
y_2delta=yt0+dydelta*delta;

% position at tf-2*delta
x_tfm2delta=xtf-dxdelta*delta;
y_tfm2delta=ytf-dydelta*delta;


% generating 4th order poly for 0-2delta, constant velocity for 2deta-(tf-2delta)
% and 4th order for (tf-2delta) to tf 

tt=t0:0.05:tf;

for ii=1:length(tt),
    
if tt(ii)<2*delta, %first 4-th poly
   xed(ii)=fourpolytraj(tt(ii),xt0,x_2delta,t0,2*delta,0,dxdelta);
   yed(ii)=fourpolytraj(tt(ii),yt0,y_2delta,t0,2*delta,0,dydelta);
elseif tt(ii)>tf-2*delta, %last 4-th poly
    xed(ii)=fourpolytraj(tt(ii),x_tfm2delta,xtf,tf-2*delta,tf,dxdelta,0);
    yed(ii)=fourpolytraj(tt(ii),y_tfm2delta,ytf,tf-2*delta,tf,dydelta,0);
else xed(ii)=xt0+dxdelta*(tt(ii)-delta);
    yed(ii)=yt0+dydelta*(tt(ii)-delta);
    %constant velocity
end
end

figure(3)
plot(tt,xed)
grid

figure(4)
plot(tt,yed)
grid


%% inverse kinematics
% end-point (xd,yd) --> joint theta1d and theta2d

for ii=1:length(tt),
    x3posd=xed(ii);
    y3posd=yed(ii);

%determine O3 location for inverse kinematics
p13=sqrt(x3posd^2+y3posd^2);
C2inv=(p13^2-a1^2-a2^2)/2/a1/a2;
C2=C2inv;

theta2invp=atan2(sqrt(1-((p13^2-a1^2-a2^2)/(2*a1*a2))^2),(p13^2-a1^2-a2^2)/(2*a1*a2));
theta2invm=-atan2(sqrt(1-((p13^2-a1^2-a2^2)/(2*a1*a2))^2),(p13^2-a1^2-a2^2)/(2*a1*a2));

theta2inv(ii)=theta2invp;% choose one

S2=sin(theta2inv(ii));
tmp=inv([a1+a2*C2 -a2*S2;a2*S2 a1+a2*C2])*[x3posd;y3posd];
theta1inv(ii)=atan2(tmp(2),tmp(1))

end

figure(5)
plot(tt,theta1inv)
grid

figure(6)
plot(tt,theta2inv)
grid

% generate time series data for simulink/simscape
theta1dsim=[tt',theta1inv']
theta2dsim=[tt',theta2inv']

