%% Author Information
% Written By: Dr. Eric Markvicka
% Modified By: Luke Freyhof
% Date: November 25, 2023

clc
clear all 
close all

l1 = 2;
l2 = 2;

c = @cos; 
s = @sin; 

%% line 
n = 3;
x = linspace(-3, 3, n);
m = 0;
y_int = 1;
y = m*x+y_int;

%% IK way points
[t1,t2] = IK(l1,l2,x,y);
theta = [t1; t2];

v_vals = zeros(size(theta));

%v_vals(:,2:n-1) = -0.5;

v_vals(1,2:n-1) = 1;
v_vals(2,2:n-1) = 0;

% time_vals = [0];
for i = 2:n
    time_vals(i) = (i-1)*3;
end
time_n = 50;


%% plot way points and geometric path
figure(1), hold on
xlim([-l1-l2, l1+l2])
ylim([-l1-l2, l1+l2])
title('Position')
plot_way_points(x,y)
plot_line(x,y)
plot_robot(l1,l2,t1,t2)


%% trajectory generation
figure(2), hold on
title('Joint Space')
plot_way_points_angles(theta, time_vals, v_vals)

for j = 1:size(theta,2)-1
    for i = 1:size(theta,1)
    
        a = find_cubic(time_vals(j),time_vals(j+1),theta(i,j), theta(i,j+1),v_vals(i,j),v_vals(i,j+1));
        
        % plot trajectory
        time = linspace(time_vals(j), time_vals(j+1), time_n);
        theta_t{i} = a(4) + a(3)*time + a(2)*time.^2 + a(1)*time.^3;
        v_t{i} = a(3) + 2*a(2)*time + 3*a(1)*time.^2;
        a_t{i} = 2*a(2) + 6*a(1)*time;
        
        figure(2)
        plot_joint_space(time, theta_t{i}, v_t{i}, a_t{i}, i)
    end

    %% actual trajectory
    for idx = 1:numel(theta_t{1})
        [x_act(idx), y_act(idx)] = FK(l1,l2,theta_t{1}(idx),theta_t{2}(idx));
    end
    
    figure(1)
    plot(x_act,y_act, 'b', 'linewidth', 2) % waypoints

end
