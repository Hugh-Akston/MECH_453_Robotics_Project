function [t1,t2] = IK(l1,l2,x,y)
% IK
r = sqrt(x.^2 + y.^2);
t2 = acos((x.^2 + y.^2 - l1^2 - l2^2)/(2*l1*l2));
t1 = atan2(y,x)-atan2(l2*sin(t2), l1 + l2*cos(t2));
end