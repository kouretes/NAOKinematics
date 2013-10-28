function [] = ForwardKinematics()
close all;
clc;
%% Nao kinematics for the left arm

global shoulderOffsetY
global elbowOffsetY
global upperArmLength
global shoulderOffsetZ
global LowerArmLength
global HandOffsetX
global HandOffsetZ
global HipOffsetZ
global HipOffsetY
global ThighLength
global TibiaLength
global FootHeight
global NeckOffsetZ

shoulderOffsetY = 98;
elbowOffsetY = 15;
upperArmLength = 105;
shoulderOffsetZ = 100;
HandOffsetX = 57.75;
HandOffsetZ = 12.31;
LowerArmLength = 55.95;
HipOffsetZ = 85; 
HipOffsetY = 50;
ThighLength = 100;
TibiaLength = 102.90;
FootHeight = 45.11;
NeckOffsetZ = 126.5;

%to d einai h y apostash apo ton proigoumeno joint
%to a einai h x apostash apo ton proigoumeno joint
%to alpha einai h diafora gwnias Z apo to proigoumeno joint
%to theta einai h gwnia tou joint

thetasL(1) = 0;
thetasL(2) = 0;
thetasL(3) = 0;
thetasL(4) = 0;
fLeftHand(thetasL);
disp('-----Left---------')
thetasL(5) = 0;
fLeftHandH25(thetasL);
disp('-----Left H25---------')

thetasR(1) = 0;
thetasR(2) = 0;
thetasR(3) = 0;
thetasR(4) = 0;
fRightHand(thetasR);
disp('-----Right---------')
thetasR(5) = 0;
fRightHandH25(thetasR);
disp('-----Right H25---------')

thetasLL(1) = 0;
thetasLL(2) = pi/2;
thetasLL(3) = 0;
thetasLL(4) = 0;%pi/2;
thetasLL(5) = 0;
thetasLL(6) = 0;
fLeftLeg(thetasLL);

thetasRL(1) = 0;
thetasRL(2) = 0;
thetasRL(3) = 0;
thetasRL(4) = 0;%+0.00000000000001;%pi/2;
thetasRL(5) = 0;
thetasRL(6) = 0;
fRightLeg(thetasRL);

thetasC(1) = 0;
thetasC(2) = pi/2;
fDownCamera(thetasC);
end


%% left hand
function [left] = fLeftHand(thetas)
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

T1 = T(0,-pi/2,0,thetas(1));
T2 = T(0,pi/2,0,thetas(2)+pi/2);
T3 = T(elbowOffsetY,pi/2,upperArmLength,thetas(3));
T4 = T(0,-pi/2,0,thetas(4));

R = Rofl(0,0,-pi/2);

Tend1 = eye(4,4);
Tend1(1,4) = HandOffsetX+LowerArmLength;
Tend1(3,4) = -HandOffsetZ;
Tend = R*Tend1;
Tendend = base*T1*T2*T3*T4*Tend;


rotZ = atan2(Tendend(2,1),Tendend(1,1));
rotY = atan2(-Tendend(3,1),sqrt(Tendend(3,2)^2 + Tendend(3,3)^2));
rotX = atan2(Tendend(3,2),Tendend(3,3));
left = [Tendend(1:3,4);rotX;rotY;rotZ];
end

%% left hand H25
function [left] = fLeftHandH25(thetas)
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
thetas(1) = pi/3
thetas(3) = pi/16;
thetas(4) = -pi/4;
thetas(5) = -pi/2
elboff = 15;

T1 = T(0,-pi/2,0,thetas(1));
T2 = T(0,pi/2,0,thetas(2)+pi/2);
T3 = T(elboff,pi/2,upperArmLength,thetas(3));
T4 = T(0,-pi/2,0,thetas(4));
%% THIS is the correct but the other is equivelant
%T5 = T(0,pi/2,LowerArmLength,thetas(5));
%Tend1 = eye(4,4);
%Tend1(1,4) = HandOffsetX;
%Tend1(3,4) = -HandOffsetZ;
%% This is to help us with the inverse kinematics, does not change ANYTHING
%% at all.
T5 = T(0,pi/2,0,thetas(5));
Tend1 = eye(4,4);
Tend1(1,4) = LowerArmLength + HandOffsetX;
Tend1(3,4) = -HandOffsetZ;
%end of fix
R = Rofl(-pi/2,0,-pi/2);
Tend = R*Tend1;
Tendend = base*T1*T2*T3*T4*T5*Tend

rotZ = atan2(Tendend(2,1),Tendend(1,1));
rotY = atan2(-Tendend(3,1),sqrt(Tendend(3,2)^2 + Tendend(3,3)^2));
rotX = atan2(Tendend(3,2),Tendend(3,3));
left = [Tendend(1:3,4);rotX;rotY;rotZ];
end

%% right hand
function [right] = fRightHand(thetas)
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

T1 = T(0,-pi/2,0,thetas(1));
T2 = T(0,pi/2,0,thetas(2)+pi/2); %To -pi/2 to afinoume panta !!!
T3 = T(-elbowOffsetY,pi/2,upperArmLength,thetas(3));
T4 = T(0,-pi/2,0,thetas(4));

R = Rofl(0,0,-pi/2);
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

%% right hand H25
function [right] = fRightHandH25(thetas)
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

T1 = T(0,-pi/2,0,thetas(1));
T2 = T(0,pi/2,0,thetas(2)+pi/2); %To -pi/2 to afinoume panta !!!
T3 = T(-elbowOffsetY,pi/2,upperArmLength,thetas(3));
T4 = T(0,-pi/2,0,thetas(4));
%% THIS is the correct but the other is equivelant
%T5 = T(0,pi/2,LowerArmLength,thetas(5));
%Tend1 = eye(4,4);
%Tend1(1,4) = HandOffsetX;
%Tend1(3,4) = -HandOffsetZ;
%% This is to help us with the inverse kinematics, does not change ANYTHING
%% at all.
T5 = T(0,pi/2,0,thetas(5));
Tend1 = eye(4,4);
Tend1(1,4) = LowerArmLength + HandOffsetX;
Tend1(3,4) = -HandOffsetZ;
%end of fix

R = Rofl(-pi/2,0,-pi/2);
Tend = R*Tend1;
Tendend = base*T1*T2*T3*T4*T5*Tend;

rotZ = atan2(Tendend(2,1),Tendend(1,1));
rotY = atan2(-Tendend(3,1),sqrt(Tendend(3,2)^2 + Tendend(3,3)^2));
rotX = atan2(Tendend(3,2),Tendend(3,3));
right = [Tendend(1:3,4);rotX;rotY;rotZ];
end

%% left leg
function [left] = fLeftLeg(thetas)
global HipOffsetZ
global HipOffsetY
global ThighLength
global TibiaLength
global FootHeight
base = eye(4,4);
base(2,4) = HipOffsetY;
base(3,4) = -HipOffsetZ;

T1 = T(0,-3*pi/4,0,thetas(1)-pi/2);
T2 = T(0,-pi/2,0,thetas(2)+pi/4);
T3 = T(0,pi/2,0,thetas(3));
T4 = T(-ThighLength,0,0,thetas(4));
T5 = T(-TibiaLength,0,0,thetas(5));
T6 = T(0,-pi/2,0,thetas(6));
R = Rofl2(pi,-pi/2,0);
Tend1 = eye(4,4);
Tend1(3,4) = -FootHeight;
Tend = R;
Tendend = base*T1*T2*T3*T4*T5*T6*Tend*Tend1;

rotZ = atan2(Tendend(2,1),Tendend(1,1));
rotY = atan2(-Tendend(3,1),sqrt(Tendend(3,2)^2 + Tendend(3,3)^2));
rotX = atan2(Tendend(3,2),Tendend(3,3));

str = sprintf('%f,%f,%f,%f,%f,%f',Tendend(1,4),Tendend(2,4),Tendend(3,4),rotX,rotY,rotZ);
%disp(str)
left = [Tendend(1:3,4);rotX;rotY;rotZ];
end

%% Right leg
function [right] = fRightLeg(thetas)
global HipOffsetZ
global HipOffsetY
global ThighLength
global TibiaLength
global FootHeight
base = eye(4,4);
base(2,4) = -HipOffsetY;
base(3,4) = -HipOffsetZ;

T1 = T(0,-pi/4,0,thetas(1)-pi/2);
T2 = T(0,-pi/2,0,thetas(2)-pi/4);
T3 = T(0,pi/2,0,thetas(3));
T4 = T(-ThighLength,0,0,thetas(4));
T5 = T(-TibiaLength,0,0,thetas(5));
T6 = T(0,-pi/2,0,thetas(6));

R = Rofl2(pi,-pi/2,0);%*Rofl(pi/2,0,0);
Tend1 = eye(4,4);
Tend1(3,4) = -FootHeight;
Tend = R;
Tendend = base*T1*T2*T3*T4*T5*T6*Tend*Tend1;


rotY = atan2(-Tendend(3,1),sqrt(Tendend(3,2)^2 + Tendend(3,3)^2));
rotZ = atan2(Tendend(2,1),Tendend(1,1));
rotX = atan2(Tendend(3,2),Tendend(3,3));

str = sprintf('%f,%f,%f,%f,%f,%f',Tendend(1,4),Tendend(2,4),Tendend(3,4),rotX,rotY,rotZ);
%disp(str)
right = [Tendend(1:3,4);rotX;rotY;rotZ];

end

%% head
function [camera] = fDownCamera(thetas)
global NeckOffsetZ
base = eye(4,4);
base(3,4) = NeckOffsetZ;
cameraZ = 23.81;
cameraX = 48.8;
T1 = T(0,0,0,thetas(1));
T2 = T(0,-pi/2,0,thetas(2)-pi/2);

R = Rofl(pi/2,pi/2,0);
Tend1 = eye(4,4);
Tend1(1,4) = cameraX;
Tend1(3,4) = cameraZ;
Tend = R;
Tendend = base*T1*T2*Tend*Tend1;
rotZ = atan2(Tendend(2,1),Tendend(1,1));
rotY = atan2(-Tendend(3,1),sqrt(Tendend(3,2)^2 + Tendend(3,3)^2));
rotX = atan2(Tendend(3,2),Tendend(3,3));
camera = [Tendend(1:3,4);rotZ;rotY;rotX];
end

function [Taf] = T(a,alpha,d,theta)
Taf = [cos(theta),            -sin(theta),            0,              a;
    sin(theta)*cos(alpha), cos(theta)*cos(alpha),  -sin(alpha),    -sin(alpha)*d;
    sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),     cos(alpha)*d;
    0,                     0,                      0,              1;];
end

function [Tranos] = Tran(x,y,z)
    Tranos = eye(4,4);
    Tranos(1,4) = x;
    Tranos(2,4) = y;
    Tranos(3,4) = z;
end

function [Ro] = Rofl(xAngle,yAngle,zAngle)
Rx = [1,                0,          0;
    0,                cos(xAngle), -sin(xAngle);
    0,                sin(xAngle), cos(xAngle);];

Ry = [cos(yAngle),       0,          sin(yAngle);
    0,                1,          0;
    -sin(yAngle),      0,          cos(yAngle);];

Rz = [cos(zAngle),       -sin(zAngle),0;
    sin(zAngle)        cos(zAngle), 0;
    0,                0,          1;];
R = Rx*Ry*Rz;

R = [R, [0;0;0];
    [0,0,0],1];
Ro = R;
end
function [Ro] = Rofl2(zAngle,yAngle,xAngle)
Rx = [1,                0,          0;
    0,                cos(xAngle), -sin(xAngle);
    0,                sin(xAngle), cos(xAngle);];

Ry = [cos(yAngle),       0,          sin(yAngle);
    0,                1,          0;
    -sin(yAngle),      0,          cos(yAngle);];

Rz = [cos(zAngle),       -sin(zAngle),0;
    sin(zAngle)        cos(zAngle), 0;
    0,                0,          1;];
R = Rz*Ry*Rx;

R = [R, [0;0;0];
    [0,0,0],1];
Ro = R;
end

