%% Author Information
% Written By: Dr. Eric Markvicka
% Modified By: Luke Freyhof
% Date: November 25, 2023

function [] = plot_way_points_angle(theta, time, v_vals)

for i = 1:size(theta,1)
    subplot(size(theta,1),2,i*2-1), hold on
    title(['joint ' num2str(i)], 'FontWeight','bold')
    xlabel('Time (s)', 'FontWeight','bold')
    ylabel('Joint angle [rad]')
    for j = 1:numel(time)
        plot(time(j),theta(i,j), 'ro', 'linewidth', 2) % waypoints
    end
    box on
end

for i = 1:size(theta,1)
    subplot(size(theta,1),2,i*2), hold on
    title(['joint ' num2str(i)], 'FontWeight','bold')
    xlabel('Time (s)', 'FontWeight','bold')
    ylabel('Joint velocity [rad/sec]')
    for j = 1:numel(time)
        plot(time(j),v_vals(i,j), 'ro', 'linewidth', 2) % waypoints
    end
    box on
end

end