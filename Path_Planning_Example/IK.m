%% Author Information
% Written By: Dr. Eric Markvicka
% Modified By: Luke Freyhof
% Date: November 25, 2023

function [t1,t2,t3] = IK(d1,l2,l3,x,y,z)
% IK
% r = sqrt(x.^2 + y.^2);
% t2 = acos((x.^2 + y.^2 - l1^2 - l2^2)/(2*l1*l2));
% t1 = atan2(y,x)-atan2(l2*sin(t2), l1 + l2*cos(t2));

t3 = acos((d1.^2 - 2.*d1.*z - l2.^2 - l3.^2 + x.^2 + y.^2 + z.^2)/(2.*l2.*l3));
t2 = atan2(z - d1, (x.^2 + y.^2).^(1/2)) + acos((d1.^2 - 2.*d1.*z + l2.^2 - l3.^2 + x.^2 + y.^2 + z.^2)./(2.*l2.*(x.^2 + y.^2 + (d1 - z).^2).^(1/2)));
t1 = atan2(y, x);
end