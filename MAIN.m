clc
clear
close all

%Lateral Dynamics Autopilot

%Most of these values are
%Taken from the past group's reports
Va=10; %m/s
b=1.48; %Wing Span
rho= 1.225; %kg/m^3 Air Density
S=.25;    %m^2 PlanForm Area

%Inertial Parameters
Jx=.028;
Jy=.026;
Jz=.053;
Jxz=0;

%Parameters that I need from the Airplane
%Stability Derivatives
Cl_p=-.4974; %Roll Damping Derivative, usually negative
Cn_p=-0.0756;  %What do all these letters mean?

%Both of these Terms are Considered Primary Control Derivative
%Can these come from aerodynamic software? 
Cl_deltaA=.1; 
Cn_deltaA=-.1;  

Gamma=Jx*Jz-Jxz^2; %Scaling Factor
Gamma3= Jz/Gamma;
Gamma4= Jxz/Gamma;
C_pp=       Gamma3*Cl_p+     Gamma4*Cn_p;
C_pdeltaA=  Gamma3*Cl_deltaA+ Gamma4*Cn_deltaA;

%%
%Define Coefficients For Transfer Function

a_phi1=-.5*rho*Va^2*S*b*C_pp*b/(2*Va)
a_phi2=.5*rho*Va^2*S*b*C_pdeltaA

%Now, use the specified parameters to define Control Gains
%As given in the BYU Book

%Design Parameters for the Response
zeta_phi=sqrt(2)/2;
deltaa_max=10; %Maximum Elevator Deflection  RADIANS OR DEGREES?
e_phimax= 1; %Maximum Error (ie maximum step size)


%Calculate Effective w_nphi
w_nphi=sqrt(abs(a_phi2)*deltaa_max/e_phimax);

kd_phi=(2*zeta_phi*w_nphi-a_phi1)/a_phi2
kp_phi=deltaa_max/e_phimax*sign(a_phi1)

%% Run the Simulation

sim('RollDynamics')

%% Plot the Result
subplot 211
plot(t,Phi,t,Desired)
legend('Actual','Command')
title('Roll Angle')

subplot 212
plot(t,Angle_Sat,t,Angle_NoSat)
title('Elevator Angle')

% 
% figure
% plot(t,Angle_Sat)


%% If the Roll Loop is Tuned, can now Determine the Dynamics of the larger X loop

g=9.81;
Vg=15; % m/s

%These Are Design Parameters for Turning the Roll Rate Loop
W_X=6; %Design Parameter, usually >5, sufficient seperation between inner and outer loop rates
zeta_X=1;

wn_X=w_nphi/W_X;

kp_X=2*zeta_X*wn_X*Vg/g
ki_X=wn_X^2*Vg/g

sim('LateralDynamics')

%% Plot the Total Results for the Lateral Dynamics
figure

plot(t,Desired,t,X)
title('Course Angle X')
xlabel('Time (s)')
ylabel('Course Angle (Radians)')
legend('Command','Response')
%% In addition, a control loop can be added on the rudder to zero out side slip. 


