%Main Longitudinal Dynamics

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
c=  S/b   ; %Chord Length? 

%Inertial Parameters
Jx=.028;
Jy=.026;
Jz=.053;
Jxz=0;

%Parameters that I need from the Airplane
%Stability Derivatives
% Cl_p=1; %Some Needed Stability Derivatives?
% Cn_p=-1;  %What do all these letters mean?

%See section 4.2.4
%Stability Derivatives, have to do with design of aircraft

%Chosen Values correpond to Va=11.6 
Cm_alpha= -1.4797      %Longitudinal Static Stability Derivative, must be less than 0
Cm_q=     17.17      %Pitch Damping Derivative

%Has to Do with Deflection of Control Surface
Cm_deltaE=-1.6591;  %Tihs term should be negative
%I took thie from the Cme term in the Bixler table.  Is that correct? 

%Both of these Terms are Considered Primary Control Derivative
%Can these come from aerodynamic software? 
% Cl_deltaA=-.1; 
% Cn_deltaA=.1;  

Gamma=Jx*Jz-Jxz^2; %Scaling Factor
Gamma3= Jz/Gamma;
Gamma4= Jxz/Gamma;
% C_pp=       Gamma3*Cl_p+     Gamma4*Cn_p;
% C_pdeltaA=  Gamma3*Cl_deltaA+ Gamma4*Cn_deltaA;

%%
%Define Coefficients For Transfer Function

% a_phi1=-.5*rho*Va^2*S*b*C_pp*b/(2*Va)
% a_phi2=.5*rho*Va^2*S*b*C_pdeltaA

%Coefficients fof Longitudinal Autopilot
%Taken from Page sectino 5.4 of Book
a_theta1=-rho*Va^2*c*S/(2*Jy)*Cm_q*c/(2*Va);
a_theta2=-rho*Va^2*c*S*Cm_alpha/(2*Jy);
a_theta3=rho*Va^2*c*S*Cm_deltaE/(2*Jy);

%Now, use the specified parameters to define Control Gains
%As given in the BYU Book

%Design Parameters for the Response
zeta_theta=sqrt(2)/2;
deltaElevator_max=10; %Maximum Elevator Deflection  RADIANS OR DEGREES?
e_thetamax= 1; %Maximum Error (ie maximum step size)


%Calculate Effective w_nphi
w_ntheta=sqrt(a_theta2+abs(a_theta3)*deltaElevator_max/e_thetamax);

%Calculate the Desired Gains
kd_theta=(2*zeta_theta*w_ntheta-a_theta1)/a_theta3
kp_theta=deltaElevator_max/e_thetamax*sign(a_theta3)

%Effective DC Gain of Inner Loop
K_thetaDC= kp_theta*a_theta3/(a_theta2+kp_theta*a_theta3)

%% Run the Simulation

sim('PitchDynamics')

%% Plot the Result
% subplot 211

plot(t,theta,t,Desired)
legend('Actual','Command')
title('Pitch Angle')

% subplot 212
% plot(t,Angle_Sat,t,Angle_NoSat)
% title('Elevator Angle')

% 
% figure
% plot(t,Angle_Sat)


%% If the Roll Loop is Tuned, can now Determine the Dynamics of the larger X loop

g=9.81;
Vg=Va;

%These Are Design Parameters for Turning the Roll Rate Loop
W_X=15; %Design Parameter, usually >5, <15, sufficient seperation between inner and outer loop rates
zeta_h=1;

wn_h=w_ntheta/W_X;

kp_h=wn_h^2/(K_thetaDC*Va)
kI_h=2*zeta_h*wn_h/(K_thetaDC*Va)

sim('FinalLongitudinalDynamics')

%% Plot the Total Results for the Lateral Dynamics
figure

plot(t,Desired,t,h)
title('Height')
xlabel('Time (s)')
ylabel('Altitude (m)')
legend('Command','Response')
%% In addition, a control loop can be added on the rudder to zero out side slip. 

















