%% Instead of Just maintaining Heading, this File will Try and Follow a Straight Line
clc
clear

%Vector Defining a Direction of the path
qe=1;
qn=1;

%Position Vector to a point on the Path
rn=2;
re=0;

Xq=atan2(qe,qn); %Desired path

Xqdeg=Xq*180/pi
%% Tunign Parameters
Kpath=.1; %A gain Governing how direct the transition will be
Xinf=pi/2; %Maximum Angle to make with desired path, maximum is pi/2


%P is the Position Vector of the Plane
pn=0;
pe=1;

%% Compute Desired Angle for a single Case
%Compute the Lateral Deviance
epy=-sin(Xq)*(pn-rn)+cos(Xq)*(pe-re)

%Compute the Course Angle to Command
Xd=-Xinf*2/pi*atan(Kpath*epy)

Xc=Xq-Xinf*2/pi*atan(Kpath*epy)

%% Plot a Full Vector Field to see what is happening
step=.1;
Range=10;
xvec=linspace(-Range, Range, 10)
yvec=xvec;
% [x,y]=meshgrid(-2:step:2, -2:step:2 )
[x,y]=meshgrid(xvec, yvec )

%Compute Error Everywhere
epsN=-sin(Xq)*(y-rn)+cos(Xq)*(x-re);  %Right now the error depends only on y...
XdN=Xq-Xinf*2/pi*atan(Kpath*epsN);
%Compute Where the Velocity should Point
L=1;
u=L*cos(XdN)
v=L*sin(XdN)

%I switched these, which might not have been the right thing to do....
%Need to think about the real angles for these.....
quiver(x,y,v,u)
xlabel('East')
ylabel('North')

%Also Plot the Desired Path
hold on 
m=qn/qe; %Slope of Real Trajectory
b=rn-m*re; %Intercept
desired=m*xvec+b;
plot(xvec,desired,'r')
xlim([-Range Range])
ylim([-Range Range])
hold off

%%
% figure
% surf(x,y,epsN)
% xlabel('East')
% ylabel('North')
% 

%Next Step Would be to wrap this control around the existing State Machine
%for Lateral Dyanmics

%I could also combine this simulation with waypoints and half plane
%transitions.


