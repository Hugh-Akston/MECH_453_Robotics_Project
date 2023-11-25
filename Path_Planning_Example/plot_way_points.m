function [] = plot_way_points(x,y)
%% plot XY
plot(x,y, 'ro', 'linewidth', 2) % waypoints
xlabel('X (mm)', 'FontWeight','bold')
ylabel('Y (mm)', 'FontWeight','bold')
box on
end