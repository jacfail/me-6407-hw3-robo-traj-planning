%% simple 3rd order polynomial trajectory

theta0=30;
thetaf=90;  %in degrees

t0=0;
tf=5;

a0=theta0;
a1=0;
a2=3/(tf^2)*(thetaf-theta0);
a3=-2/(tf^3)*(thetaf-theta0);

tt=t0:0.1:tf;

thetadata=a0+a1*tt+a2*tt.^2+a3*tt.^3;
veldata=a1+2*a2*tt+3*a3*tt.^2;
accdata=2*a2+6*a3*tt;

%% ploting

plot(tt,thetadata)

grid





