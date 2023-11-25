%% Author Information
% Written By: Dr. Eric Markvicka
% Modified By: Luke Freyhof
% Date: November 25, 2023

function [] = plot_joint_space(time, theta_t, v_t, a_t, theta_num)

subplot(2,2,theta_num*2-1) % position
plot(time, theta_t, '--r', 'linewidth', 2)

subplot(2,2,theta_num*2) % position
plot(time, v_t, '--r', 'linewidth', 2)

end