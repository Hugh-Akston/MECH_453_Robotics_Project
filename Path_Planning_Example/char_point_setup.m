function [char_points] = char_point_setup(points,char_index)
%char_point_setup

char_points = zeros(length(char_index), 3);

for i = 1:length(char_index)
    char_points(i, 1) = points(char_index(i), 2); % Create the x column of the list of points used to draw the character
    char_points(i, 2) = points(char_index(i), 3); % Create the y column of the list of points used to draw the character
    char_points(i, 3) = points(char_index(i), 4); % Create the z column of the list of points used to draw the character
end

end