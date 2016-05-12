%Prototype of Line Follow Code.......   What is the matter?

clc
clear
WayPoint_Index=1


while (1)

% Lake Lag Coordinates (E,N)
LakeLag=[-173,  143;
-82,   228;
176,    81;
181,  -138;
54,   -148;
62,   -219;
-36,  -216;
-101, -142;
-181, -112;
-173, 143];


%Vector of Waypoints in N, E Form
Waypoints=[150, 0.0; 
    0, 100;
    -100, 100;
    0, -100;
    1001,1001];

Waypoints2=[50.0, 150.0;
            50,-200;
            1001, 1001];
Waypoints3= [-50.0  ,  100.0;
                      -50.0  , 0.0  ;
                      0.0  , -75.0  ;
                      100.0  , -75.0  ;  
                      150.0  , 0.0   ;
                      100.0  , 100.0   ;
                      50.0  , 100.0  ;
                      -47.0  , 74.0  ;
                      -134.0  , 24.0  ;
                      1001.0  , 1001.0 ];
                     

plot(LakeLag(:,1),LakeLag(:,2),'b',Waypoints(1:end-1,2),Waypoints(1:end-1,1),'r')

[PositionE,PositionN]=ginput(1);
%%

[Xc,WayPoint_Index]=Straight_Line(Waypoints, WayPoint_Index,PositionN,PositionE)


Xcindeg=Xc*180/pi


end










