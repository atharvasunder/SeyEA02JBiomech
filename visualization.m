f1 = figure;
f2 = figure;
f3 = figure;

figure(f1);

imagesc(attack_angle, k, output_1);  % Create heatmap

hold on;

% Compute the expression 1600 / (1 - sin(attack_angle))
reference_function_vals = 1600 ./ (1 - sind(attack_angle));  % Use 'sind' for sine of angles in degrees

plot(attack_angle,reference_function_vals,"Marker",".","LineWidth",2)

colorbar;  % Show color scale
ylabel('Spring constant k (N/m)');
xlabel('Attack angle (degrees)');
title('Steps to fall (Vx (t=0) = 5m/s) ');

% Correct the y-axis direction
set(gca, 'YDir', 'normal');  % This makes the y-axis go from min to max

% Optionally, you can adjust the color map
colormap('cool');  % Use the 'hot' colormap (you can try other colormaps too)



figure(f2);

imagesc(k, initial_velocity_x, output_2);  % Create heatmap

colorbar;  % Show color scale
ylabel('Initial horizontal velocity (m/s)');
xlabel('Spring constant k (N/m)');
title('Steps to fall (attack angle = 68 deg) ');

% Correct the y-axis direction
set(gca, 'YDir', 'normal');  % This makes the y-axis go from min to max

% Optionally, you can adjust the color map
colormap('cool');  % Use the 'hot' colormap (you can try other colormaps too)



figure(f3);

imagesc(attack_angle, initial_velocity_x, output_3);  % Create heatmap

colorbar;  % Show color scale
ylabel('Initial horizontal velocity (m/s)');
xlabel('Attack angle (degrees)');
title('Steps to fall (k = 20 kN/m) ');

% Correct the y-axis direction
set(gca, 'YDir', 'normal');  % This makes the y-axis go from min to max

% Optionally, you can adjust the color map
colormap('cool');  % Use the 'hot' colormap (you can try other colormaps too)