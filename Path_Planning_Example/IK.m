%% Author Information
% Written By: Dr. Eric Markvicka
% Modified By: Luke Freyhof
% Date: November 25, 2023

function [t1,t2,t3, theta] = IK(d1,l2,l3,char_waypoints)
% IK

x = char_waypoints(:, 1); % Extract the x values from the waypoints array
y = char_waypoints(:, 2); % Extract the y values from the waypoints array
z = char_waypoints(:, 3); % Extract the z values from the waypoints array

t3 = acos((d1.^2 - 2.*d1.*z - l2.^2 - l3.^2 + x.^2 + y.^2 + z.^2)/(2.*l2.*l3));
t2 = atan2(z - d1, (x.^2 + y.^2).^(1/2)) + acos((d1.^2 - 2.*d1.*z + l2.^2 - l3.^2 + x.^2 + y.^2 + z.^2)./(2.*l2.*(x.^2 + y.^2 + (d1 - z).^2).^(1/2)));
t1 = atan2(y, x);

theta = [t1 t2 t3]; % Combine the joint angles into one array
end