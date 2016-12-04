clc;
clear;

%Part(i)
%Develop software coding in Matlab to compute the inverse kinematic
%solutions for all possible arm configurations so as to reach the target 
%position and orientation.

%(Px_arm,Py_arm,Pz_arm)T = (Px,Py,Pz)T - d6(ax,ay,az)T;
Pxyz = [1,1,0]';      
d6 = 0.5;
n = [-1/sqrt(2),0, 1/sqrt(2)]';
s = [0, -1, 0]';
a = [1/sqrt(2), 0, 1/sqrt(2)]';     %(ax,ay,az)T;

P_arm = Pxyz - d6 * a;     %(Px_arm,Py_arm,Pz_arm)T

%other configuration
d2 = 0.25;
d4 = 1;
a2 = 0.5;

%Joint angle 1 right
Angle1 = atan2d(P_arm(2),P_arm(1)) - atan2d(d2,sqrt(P_arm(1)^2 + P_arm(2)^2 - d2^2));

%Joint angle 2
A_Temp = cosd(Angle1)*P_arm(1) + sind(Angle1)*P_arm(2);
B_Temp = ((A_Temp^2) + (P_arm(3)^2) + (a2^2) - (d4^2)) / (2*a2);

Angle2 = atan2d(A_Temp,P_arm(3)) - atan2d(B_Temp,sqrt(A_Temp^2 + P_arm(3)^2 - B_Temp^2)); 

%Joint angle 3
Angle3 = atan2d(A_Temp - a2*cosd(Angle2), P_arm(3)+a2*sind(Angle2)) - Angle2;

%Joint angle 4
Angle4 = atan2d(-sind(Angle1)*a(1) + cosd(Angle1)*a(2),cosd(Angle2+Angle3)*(cosd(Angle1)*a(1) + sind(Angle1)*a(2)) - sind(Angle2+Angle3)*a(3));

%Joint angle 5
Angle5 = atan2d(sqrt(((cosd(Angle1)*cosd(Angle2+Angle3)*a(1)+sind(Angle1)*cosd(Angle2+Angle3)*a(2)-sind(Angle2+Angle3)*a(3))^2) + ...,
    (-sind(Angle1)*a(1)+cosd(Angle1)*a(2))^2 ),(sind(Angle2+Angle3)*(cosd(Angle1)*a(1)+sind(Angle1)*a(2))+cosd(Angle2+Angle3)*a(3)));
                    
%Joint angle 6
Angle6 = atan2d((sind(Angle2+Angle3)*(cosd(Angle1)*s(1) + sind(Angle1)*s(2)) + cosd(Angle2+Angle3)*s(3)), ...,
    -(sind(Angle2+Angle3)*(cosd(Angle1)*n(1)+sind(Angle1)*n(2))+cosd(Angle2+Angle3)*n(3)));

JointAngle = zeros(1,6);        %Joint Angle
 JointAngle(1) = Angle1;
 JointAngle(2) = Angle2;
 JointAngle(3) = Angle3;
 JointAngle(4) = Angle4;
 JointAngle(5) = Angle5;
 JointAngle(6) = Angle6;
disp('JointAngle(degree)');
disp(JointAngle);

%Part(ii)
%validate the accuracy of all the inverse kinematics solutions obtained in
%Part(i)using the forward kinematics and display the following results:

d_offset = [0,0.25,0,1,0,0.5];  %Offset Distance di(m)
a_LinkLength = [0,0.5,0,0,0,0];    %Link length
a_TwistAngle = [-90,0,90,-90,90,0];  %Twist Angle
A = zeros(4,4,6);
R = zeros(3,3,6);

for n = 1:6
    A(:,:,n) = [cosd(JointAngle(n)),-cosd(a_TwistAngle(n))*sind(JointAngle(n)), sind(a_TwistAngle(n))*sind(JointAngle(n)), a_LinkLength(n)*cosd(JointAngle(n)); ...,
        sind(JointAngle(n)), cosd(JointAngle(n))*cosd(a_TwistAngle(n)) , -sind(a_TwistAngle(n))*cosd(JointAngle(n)), a_LinkLength(n)*sind(JointAngle(n)); ...,
        0, sind(a_TwistAngle(n)), cosd(a_TwistAngle(n)), d_offset(n); ...,
        0,0,0,1];
end
disp('Homogeneous transformation matrices A(i-1)(i)');
disp(A);

T06 = 1;
for m = 1:6
    T06 = T06 * A(:,:,m);
end
disp('T06');
disp(T06);

for n = 1:6
    R(:,:,n) = A(1:3,1:3,n);
end
R03 = R(:,:,1)* R(:,:,2)* R(:,:,3);
R36 = R(:,:,4)* R(:,:,5)* R(:,:,6);
R06 = R03*R36;
disp('R03');
disp(R03);
disp('R36');
disp(R36);
disp('R06');
disp(R06);



