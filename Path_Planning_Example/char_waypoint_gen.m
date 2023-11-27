function [char_waypoints] = char_waypoint_gen(char_points, n)
% char_waypoint_gen

char_waypoints = zeros(length(char_points) + (n - 2)*(length(char_points) - 1), 3); % Preallocate size of coordinates array including waypoints

for i = 1:length(char_points) % - 1
    char_waypoints((i + (i - 1)*(n - 2)), 1) = char_points(i, 1); % Assign the original coordinate values to the x column in the waypoints array.  This spreads out the original points and leaves space for the new waypoints.
    char_waypoints((i + (i - 1)*(n - 2)), 2) = char_points(i, 2); % Assign the original coordinate values to the y column in the waypoints array.  This spreads out the original points and leaves space for the new waypoints.
    char_waypoints((i + (i - 1)*(n - 2)), 3) = char_points(i, 3); % Assign the original coordinate values to the z column in the waypoints array.  This spreads out the original points and leaves space for the new waypoints.
end

end