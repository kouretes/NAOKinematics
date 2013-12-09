function [Tendend right] = fRightHand(thetas)
global shoulderOffsetY
global LowerArmLength
global elbowOffsetY
global shoulderOffsetZ
global HandOffsetX
global upperArmLength
global HandOffsetZ
base = eye(4,4);
base(2,4) = -shoulderOffsetY;
base(3,4) = shoulderOffsetZ;

T1 = DH(0,-pi/2,0,thetas(1));
T2 = DH(0,pi/2,0,thetas(2)+pi/2); %To -pi/2 to afinoume panta !!!
T3 = DH(-elbowOffsetY,pi/2,upperArmLength,thetas(3));
T4 = DH(0,-pi/2,0,thetas(4));

R = RotXYZMatrix(0,0,-pi/2);
Tend1 = eye(4,4);
Tend1(1,4) = HandOffsetX+LowerArmLength;
Tend1(3,4) = -HandOffsetZ;
Tend = R*Tend1;
Tendend = base*T1*T2*T3*T4*Tend;


rotZ = atan2(Tendend(2,1),Tendend(1,1));
rotY = atan2(-Tendend(3,1),sqrt(Tendend(3,2)^2 + Tendend(3,3)^2));
rotX = atan2(Tendend(3,2),Tendend(3,3));
right = [Tendend(1:3,4);rotX;rotY;rotZ];
end
