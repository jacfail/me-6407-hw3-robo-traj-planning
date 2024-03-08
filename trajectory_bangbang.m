%% Bang-bang control trajectory



theta0=30;
thetaf=90;  %in degrees

accmax=14;

t0=0;
tf=2*sqrt((thetaf-theta0)/accmax);

tt=t0:0.1:tf;

thetadata_firsthalf=theta0+(1/2*accmax*tt.^2);
thetadata_firsthalf=thetadata_firsthalf.*(tt<tf/2);

thetadata_secondhalf=theta0+(accmax*(tf/2)^2)-(1/2*accmax*(tf-tt).^2);
thetadata_secondhalf=thetadata_secondhalf.*(tt>tf/2);

thetadata=thetadata_firsthalf+thetadata_secondhalf;
%% ploting

plot(tt,thetadata)

grid
