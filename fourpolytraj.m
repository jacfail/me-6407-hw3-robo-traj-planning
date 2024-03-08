%% 4th-order polynomial trajectory 
%initial and final acc=0
%boundary condition: (xe-x0)=tf/2*(dxf-dx0)
function y=fourpolytraj(t,x0,xf,t0,tf,dx0,dxf)

%check if t is between t0 and tf
if t>tf | t<t0
    disp('time is out of range')
end

t=t-t0;
tf=tf-t0;

a0=x0;
a1=dx0;
a2=0;
a3=1/2/tf^3*(20*xf-20*x0-(8*dxf+12*dx0)*tf);
a4=1/2/tf^4*(30*x0-30*xf+(14*dxf+16*dx0)*tf);

y=a0+a1*t+a2*t.^2+a3*t.^3+a4*t.^4;
