function [Tendend camera Derivatives] = fDownCamera(thetas)
global NeckOffsetZ

Derivatives=zeros(2,4,4);
base = eye(4,4);
base(3,4) = NeckOffsetZ;
cameraZ = 23.81;
cameraX = 48.8;
T1 = DH(0,0,0,thetas(1));
dT1= DHDerivative(0,0,0,thetas(1));
T2 = DH(0,-pi/2,0,thetas(2)-pi/2);
dT2= DHDerivative(0,-pi/2,0,thetas(2)-pi/2);


Tend = RotXYZMatrix(pi/2,pi/2,0);

Tend1 = eye(4,4);
Tend1(1,4) = cameraX;
Tend1(3,4) = cameraZ;

Tendend = base*T1*T2*Tend*Tend1;
Derivatives(1,:,:)=base*dT1*T2*Tend*Tend1; %Partial Derivatives
Derivatives(2,:,:)=base*T1*dT2*Tend*Tend1; %Partial Derivatives

rotZ = atan2(Tendend(2,1),Tendend(1,1));
rotY = atan2(-Tendend(3,1),sqrt(Tendend(3,2)^2 + Tendend(3,3)^2));
rotX = atan2(Tendend(3,2),Tendend(3,3));
camera = [Tendend(1:3,4);rotZ;rotY;rotX];
end
