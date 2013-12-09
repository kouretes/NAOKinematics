function [Taf] = DH(a,alpha,d,theta)
Taf = [cos(theta),            -sin(theta),            0,              a;
    sin(theta)*cos(alpha), cos(theta)*cos(alpha),  -sin(alpha),    -sin(alpha)*d;
    sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),     cos(alpha)*d;
    0,                     0,                      0,              1;];
end
