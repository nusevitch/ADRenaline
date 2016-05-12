function [ Xc, WayPoint_Index ] = Straight_Line( Waypoint, WayPoint_Index, position_N, position_E )
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here

Max_Line_Angle=pi/4;  %These values are set from Q ground control
K_Line_Follow=.1;       %ON pixhawk, these are set from Q ground

    qN=Waypoint(WayPoint_Index+1,1)-Waypoint(WayPoint_Index,1); %//First Index needs a plus one
    qE=Waypoint(WayPoint_Index+1,2)-Waypoint(WayPoint_Index,2); %//First in
    pNC=position_N-Waypoint(WayPoint_Index+1,1);  %Order of these
    pEC=position_E-Waypoint(WayPoint_Index+1,2);

    if(qN*pNC+qE*pEC>0.0)  %Direction of this changed
        WayPoint_Index=WayPoint_Index+1;
        if (Waypoint(WayPoint_Index+1,1)>1000)  %Remember the alert at the end of waypoints
            WayPoint_Index=1;       
        end

        qN=Waypoint(WayPoint_Index+1,1)-Waypoint(WayPoint_Index,1); %//First Index needs a plus one
        qE=Waypoint(WayPoint_Index+1,2)-Waypoint(WayPoint_Index,2); %//First in
    end


%     pN=position_N-Waypoint(WayPoint_Index,1);
%     pE=position_E-Waypoint(WayPoint_Index,2);
    



    %//Components in Both Directions
    %//Do I need to use pow or something to get these values?
    %//I think no need to normalize..., all I need is the angle
    %//qN=YComp/((Ycomp^2+XComp^2))^(0.5);  //Data Type Issues?
    %//qE=YComp/((Ycomp^2+XComp^2))^(0.5);  %
    %//These are the Position Variables the I have access to
    %//float position_N = 0.0f;
    %//float position_E = 0.0f;
    Xq=atan2(qE,qN); %//Desired path (Angle from North)
    % Tunign Parameters, Need to get these from Q Ground Control
    %//Kpath=.1; //A gain Governing how direct the transition will be
    %/%/Xinf=3.14159/2.0; //Maximum Angle to make with desired path, maximum is pi/2
    %//%Compute the Lateral Distance from the Correct Line
    epy=-sin(Xq)*(position_N- Waypoint(WayPoint_Index,1))+cos(Xq)*(position_E-Waypoint(WayPoint_Index,2));
%     //Compute the Commanded Course Angle
   
    Xc=Xq-Max_Line_Angle*2.0/3.14159*atan(K_Line_Follow*epy);
%     return Xc;
    if Xc>pi
       Xc=Xc-2*pi; 
    end
    
%    Xc=Xq-Max_Line_Angle*2.0/3.14159*atan(K_Line_Follow*epy);
    
    
end

