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
disp('-----Left---------')
thetasL(1) = 0;
thetasL(2) = 0;
thetasL(3) = 0;
thetasL(4) = 0;
fLeftHand(thetasL);

disp('-----Left H25---------')
thetasL(5) = 0;
fLeftHandH25(thetasL);

disp('-----Right---------')
thetasR(1) = 0;
thetasR(2) = 0;
thetasR(3) = 0;
thetasR(4) = 0;
fRightHand(thetasR);

disp('-----Right H25---------')
thetasR(5) = 0;
fRightHandH25(thetasR);

disp('-----Left Leg---------')
thetasLL(1) = 0;
thetasLL(2) = pi/2;
thetasLL(3) = 0;
thetasLL(4) = 0;%pi/2;
thetasLL(5) = 0;
thetasLL(6) = 0;
fLeftLeg(thetasLL);


disp('-----Right Leg---------')
thetasRL(1) = 0.4;
thetasRL(2) = 0.1;
thetasRL(3) = 0.2;
thetasRL(4) = 0.3;%+0.00000000000001;%pi/2;
thetasRL(5) = 0.4;
thetasRL(6) = 0.5;
[T v v]=fRightLeg(thetasRL);
T(3,4)+=1;
r=JacobianInverse(T,@fRightLeg,zeros(6,1));%thetasRL'+2*pi+randn(6,1)*0.001);
fRightLeg(thetasRL)*fRightLeg(r)^-1
r-thetasRL'

pause
disp('-----Camera---------')
%thetasC(1) = 0;
%thetasC(2) = pi/2;
%[T v v]=fDownCamera(thetasC);
%r=JacobianInverse(T,@fDownCamera,thetasC'+randn(2,1)*0.01);
%r-thetasC'

end







