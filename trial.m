% % Define a grid of x and y values
% [x, y] = meshgrid(linspace(-5, 5, 50), linspace(-5, 5, 50));
% 
% % Define the function f(x, y)
% fxy = sin(x) .* cos(y);
% 
% % Flatten the x, y, and f(x, y) arrays for plotting
% x_flat = x(:);
% y_flat = y(:);
% fxy_flat = fxy(:);
% 
% % Create a scatter plot
% scatter(x_flat, y_flat, 100, fxy_flat, 'filled');
% 
% % Add a colorbar to show the color mapping
% colorbar;
% 
% % Add labels and title
% xlabel('x');
% ylabel('y');
% title('Scatter Plot with Shading Based on f(x, y)');

% Define a grid of x and y values
x = linspace(-5, 5, 50);
y = linspace(-5, 5, 50);

% Define the function f(x, y)
fxy = sin(x') * cos(y); % Outer product to get 2D function values

% Create a heatmap
imagesc(x, y, fxy);

% Set the color scale
colorbar;

% Add labels and title
xlabel('x');
ylabel('y');
title('Heatmap of f(x, y)');

