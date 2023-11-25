function [] = plot_robot(l1,l2,t1,t2)

c = @cos; 
s = @sin; 

% 
startColor = [0.5, 0.5, 0.5];
for i = 1:numel(t1)
    x = [0 l1*c(t1(i)) l1*c(t1(i)) + l2*c(t1(i)+t2(i))];
    y = [0 l1*s(t1(i)) l1*s(t1(i)) + l2*s(t1(i)+t2(i))];
    plot(x, y, 'Color', startColor + (i-1) / 40, 'LineWidth',2);
end

end