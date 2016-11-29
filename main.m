clc;
clear;

%Develop software coding in Matlab to compute the inverse kinematic
%solutions for all possible arm configurations so as to reach the target 
%position and orientation.

%(Px_arm,Py_arm,Pz_arm)T = (Px,Py,Pz)T - d6(ax,ay,az)T;
P = [1,1,0]';       %(Px_arm,Py_arm,Pz_arm)T
d6 = 0.5;
n = [-1/2^(1/2),0, 1/2^(1/2)]';
s = [0, -1, 0]';
a = [1/2^(1/2), 0, 1/2^(1/2)]';     %(ax,ay,az)T;

P_arm = P - d6 * a;     %(Px_arm,Py_arm,Pz_arm)T

%other configuration
d2 = 0.25;
d4 = 1;
a2 = 0.5;

%Joint angle 1 right
Angle1 = atan2d(P_arm(2),P_arm(1)) - atan2d(d2,(P_arm(1)^2 + P_arm(2)^2 - d2^2)^(1/2));
%JoAngle1_l = atan2d(P_arm(2),P_arm(1)) - atan2d(d2,-(P_arm(1)^2 + P_arm(2)^2 - d2^2)^(1/2));

%Joint angle 2
A = cosd(Angle1)*P_arm(1) + sind(Angle1)*P_arm(2);
B = (A^2 + P_arm(3)^2 + a2^2 - d4^2) / 2*a2;

Angle2 = atan2d(A,P_arm(3)) - atan2d(B,(A^2 + P_arm(3)^2 - B^2)); %elbow-down

%Joint angle 3
Angle3 = atan2d(A - a2*cosd(Angle2), P_arm(3)+a2*sind(Angle2)) - Angle2;

%Joint angle 4
Angle4 = atan2d(-sind(Angle1)*a(1) + cosd(Angle1)*a(2), cosd(Angle2+Angle3)*(cosd(Angle1)*a(1) + sind(Angle1)*a(2)) - sind(Angle2+Angle3)*a(3));

%Joint angle 5
Angle5 = atan2d(((cosd(Angle1)*cosd(Angle2+Angle3)*a(1)+sind(Angle1)*cosd(Angle2+Angle3)*a(2)-sind(Angle2+Angle3)*a(3))^2 + ...,
    (-sind(Angle1)*a(1)+cosd(Angle1)*a(2))^2 )^(1/2),sind(Angle2+Angle3)*(cosd(Angle1)*a(1)+sind(Angle1)*a(2))+cosd(Angle2+Angle3)*a(3));
                    
%Joint angle 6
Angle6 = atan2d( sind(Angle2+Angle3)*(cosd(Angle1)*s(1) + sind(Angle1)*s(2)) + cosd(Angle2+Angle3)*s(3), ...,
    -(sind(Angle2+Angle3)*(cosd(Angle1)*n(1)+sind(Angle1)*n(2))+cosd(Angle2+Angle3)*n(3)));




