function [char_waypoints] = char_waypoint_assign(char_points, char_waypoints, n)
% char_waypoint_assign

x_increment = zeros(length(char_points) - 1); % Preallocate the size for the x_increment vector
y_increment = zeros(length(char_points) - 1); % Preallocate the size for the y_increment vector
z_increment = zeros(length(char_points) - 1); % Preallocate the size for the z_increment vector

for i = 1:length(char_points) - 1
    x_increment(i) = (char_points(i+1, 1) - char_points(i, 1))/(n - 1); % Find the distance that should be between each set of waypoints
    for j = 1:(n - 2)
        char_waypoints((i + (i - 1)*(n - 2)) + j, 1) = char_points(i, 1) + j*(x_increment(i)); % Add the waypoints between the original values into the waypoint array
    end

    y_increment(i) = (char_points(i+1, 2) - char_points(i, 2))/(n - 1); % Find the distance that should be between each set of waypoints
    for j = 1:(n - 2)
        char_waypoints((i + (i - 1)*(n - 2)) + j, 2) = char_points(i, 2) + j*(y_increment(i)); % Add the waypoints between the original values into the waypoint array
    end

    z_increment(i) = (char_points(i+1, 3) - char_points(i, 3))/(n - 1); % Find the distance that should be between each set of waypoints
    for j = 1:(n - 2)
        char_waypoints((i + (i - 1)*(n - 2)) + j, 3) = char_points(i, 3) + j*(z_increment(i)); % Add the waypoints between the original values into the waypoint array
    end
end

end