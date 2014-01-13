function [Taf] = DHDerivative(a,alpha,d,theta)
Taf = [-sin(theta),			-cos(theta),       		0,	0;
	cos(theta)*cos(alpha),	-sin(theta)*cos(alpha),	0,	0;
	cos(theta)*sin(alpha),	-sin(theta)*sin(alpha),	0,	0;
	0,						0,                    	0,	0;];
    
    %Taf=Taf* DH(a,alpha,d,theta)^-1;
end
