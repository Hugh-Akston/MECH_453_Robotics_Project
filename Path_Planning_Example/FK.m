function [x,y] = FK(l1,l2,t1,t2)
c = @cos; 
s = @sin; 

% FK
x = l1*c(t1) + l2*c(t1+t2);
y = l1*s(t1) + l2*s(t1+t2);
end