function [Tendend left] = fLeftHandH25(thetas)
global shoulderOffsetY
global shoulderOffsetZ
global elbowOffsetY
global LowerArmLength
global HandOffsetX
global upperArmLength
global HandOffsetZ
base = eye(4,4);
base(2,4) = shoulderOffsetY;
base(3,4) = shoulderOffsetZ;

elboff = 15;

T1 = DH(0,-pi/2,0,thetas(1));
T2 = DH(0,pi/2,0,thetas(2)+pi/2);
T3 = DH(elboff,pi/2,upperArmLength,thetas(3));
T4 = DH(0,-pi/2,0,thetas(4));
%% THIS is the correct but the other is equivelant
%T5 = T(0,pi/2,LowerArmLength,thetas(5));
%Tend1 = eye(4,4);
%Tend1(1,4) = HandOffsetX;
%Tend1(3,4) = -HandOffsetZ;
%% This is to help us with the inverse kinematics, does not change ANYTHING
%% at all.
T5 = DH(0,pi/2,0,thetas(5));
Tend1 = eye(4,4);
Tend1(1,4) = LowerArmLength + HandOffsetX;
Tend1(3,4) = -HandOffsetZ;
%end of fix
R = RotXYZMatrix(-pi/2,0,-pi/2);
Tend = R*Tend1;
Tendend = base*T1*T2*T3*T4*T5*Tend;

rotZ = atan2(Tendend(2,1),Tendend(1,1));
rotY = atan2(-Tendend(3,1),sqrt(Tendend(3,2)^2 + Tendend(3,3)^2));
rotX = atan2(Tendend(3,2),Tendend(3,3));
left = [Tendend(1:3,4);rotX;rotY;rotZ];
end
