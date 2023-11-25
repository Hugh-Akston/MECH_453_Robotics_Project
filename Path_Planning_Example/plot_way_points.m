%% Author Information
% Written By: Dr. Eric Markvicka
% Modified By: Luke Freyhof
% Date: November 25, 2023

function [] = plot_way_points(x,y)
%% plot XY
plot(x,y, 'ro', 'linewidth', 2) % waypoints
xlabel('X (mm)', 'FontWeight','bold')
ylabel('Y (mm)', 'FontWeight','bold')
box on
end