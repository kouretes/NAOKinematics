function [Taf] = DHDerivative(a,alpha,d,theta)
Taf = [-sin(theta),            -cos(theta),            0,              a;
    cos(theta)*cos(alpha), -sin(theta)*cos(alpha),  -sin(alpha),    -sin(alpha)*d;
    cos(theta)*sin(alpha), -sin(theta)*sin(alpha),  cos(alpha),     cos(alpha)*d;
    0,                     0,                      0,              1;];
    
    %Taf=Taf* DH(a,alpha,d,theta)^-1;
end
