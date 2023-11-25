%% Author Information
% Written By: Dr. Eric Markvicka
% Modified By: Luke Freyhof
% Date: November 25, 2023

function [a] = find_cubic(time_1,time_f,t1,t2,v0,vf)

% coefficients 
t0 = time_1;
tf = time_f;
A = [t0^3 t0^2 t0 1;
     tf^3 tf^2 tf 1;
     3*t0^2 2*t0 1 0;
     3*tf^2 2*tf 1 0];
b = [t1; t2; v0; vf];

a = A^-1*b;

end