function [Tendend left] = fLeftLeg(thetas)
global HipOffsetZ
global HipOffsetY
global ThighLength
global TibiaLength
global FootHeight
base = eye(4,4);
base(2,4) = HipOffsetY;
base(3,4) = -HipOffsetZ;

T1 = DH(0,-3*pi/4,0,thetas(1)-pi/2);
T2 = DH(0,-pi/2,0,thetas(2)+pi/4);
T3 = DH(0,pi/2,0,thetas(3));
T4 = DH(-ThighLength,0,0,thetas(4));
T5 = DH(-TibiaLength,0,0,thetas(5));
T6 = DH(0,-pi/2,0,thetas(6));
R = RotZYXMatrix(pi,-pi/2,0);
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
