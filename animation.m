tic % Starts clock in matlab

%% Initializing plots

f1 = figure;
set(f1, 'Color', 'w'); % Set background color to white
set(f1, 'DefaultTextInterpreter', 'latex');

f2 = figure;
set(f2, 'Color', 'w'); % Set background color to white
set(f2, 'DefaultTextInterpreter', 'latex');

f3 = figure;
set(f3, 'Color', 'w'); % Set background color to white
set(f3, 'DefaultTextInterpreter', 'latex');

f4 = figure;
set(f4, 'Color', 'w'); % Set background color to white
set(f4, 'DefaultTextInterpreter', 'latex');

t = 0;      % Instantous time, initialized to 0

figure(f1)   % Creates a window where the graph will be plotted

com_position = plot(start_x, start_y, 'o');  % Center of mass position

hold on;

com_position_variation = plot(start_x, start_y, 'blue','linewidth',1.5);

title("COM position in single leg running");
xlabel("x(t) (m)");
ylabel("y(t) (m)");

xlim([0,15]);
ylim([0,3]);

grid on;
box on;

set(gca, 'FontSize', 12);
set(gca, 'Box', 'on');
set(gca, 'LineWidth', 1.2);

figure(f2)

com_velocity = plot(start_vx, start_vy, 'o');

hold on;

com_velocity_variation = plot(start_vx, start_vy, 'green','linewidth',1.5);

title("COM velocity in single leg running");
xlabel("vx(t)");
ylabel("vy(t)");

xlim([-5,5]);
ylim([-5,5]);

grid on;
box on;

set(gca, 'FontSize', 12);
set(gca, 'Box', 'on');
set(gca, 'LineWidth', 1.2);

figure(f3)

t_values = final_times;
x_values = final_solution(1,:);
y_values = final_solution(2,:);
vx_values = final_solution(3,:);
vy_values = final_solution(4,:);
ax_values = diff(vx_values)./diff(t_values);  % Reduces the length of array by 1 because differences are taken
ax_values = [ax_values , ax_values(end)];
ay_values = diff(vy_values)./diff(t_values);
ay_values = [ay_values , ay_values(end)];

subplot(3,2,1)

x_vs_time = plot(t_values, x_values, 'blue','linewidth',1.5);

title("position x component vs time");
xlabel("t");
ylabel("x (m)");

xlim([0,t_end]);
ylim([0,15]);

grid on;
box on;

set(gca, 'FontSize', 12);
set(gca, 'Box', 'on');
set(gca, 'LineWidth', 1.2);

subplot(3,2,2)

y_vs_time = plot(t_values, y_values, 'blue','linewidth',1.5);

hold on

plot(t_values, og_y_land*ones(size(t_values)), 'red')

title("position y component vs time");
xlabel("t");
ylabel("y (m)");

xlim([0,t_end]);
ylim([0,1.5]);

grid on;
box on;

set(gca, 'FontSize', 12);
set(gca, 'Box', 'on');
set(gca, 'LineWidth', 1.2);

subplot(3,2,3)

vx_vs_time = plot(t_values, vx_values, 'red','linewidth',1.5);

title("velocity x component vs time");
xlabel("t");
ylabel("vx (m/s)");

xlim([0,t_end]);
ylim([4,6]);

grid on;
box on;

set(gca, 'FontSize', 12);
set(gca, 'Box', 'on');
set(gca, 'LineWidth', 1.2);

subplot(3,2,4)

vy_vs_time = plot(t_values, vy_values, 'red','linewidth',1.5);

title("velocity y component vs time");
xlabel("t");
ylabel("vy (m/s)");

xlim([0,t_end]);
ylim([-3,3]);

grid on;
box on;

set(gca, 'FontSize', 12);
set(gca, 'Box', 'on');
set(gca, 'LineWidth', 1.2);

subplot(3,2,5)

ax_vs_time = plot(t_values, ax_values, 'green','linewidth',1.5);

title("acceleration x component vs time");
xlabel("t");
ylabel("ax (m/s2)");

xlim([0,t_end]);
ylim([-20,20]);

grid on;
box on;

set(gca, 'FontSize', 12);
set(gca, 'Box', 'on');
set(gca, 'LineWidth', 1.2);

subplot(3,2,6)

ay_vs_time = plot(t_values, ay_values, 'green','linewidth',1.5);

hold on

plot(t_values, -g*ones(size(t_values)), 'red')

title("acceleration y component vs time");
xlabel("t");
ylabel("ay (m/s2)");

xlim([0,t_end]);
ylim([-20,20]);

grid on;
box on;

set(gca, 'FontSize', 12);
set(gca, 'Box', 'on');
set(gca, 'LineWidth', 1.2);

figure(f4)

TE = KE + PE;   % Total energy

KE_vs_t = plot(t_values,KE,'blue','linewidth',1,DisplayName='KE');

hold on

PE_vs_t = plot(t_values,PE,'red','linewidth',1,DisplayName='PE');

TE_vs_t = plot(t_values,TE,'yellow','linewidth',1.5,DisplayName='TE');

title("Energy variation in single leg running");
xlabel("t (s)");
ylabel("KE , PE , TE");

legend('show')

% xlim([-15,15]);
% ylim([-15,15]);

grid on;
box on;

set(gca, 'FontSize', 12);
set(gca, 'Box', 'on');
set(gca, 'LineWidth', 1.2);
%% Animation starts here

while t<t_end
    
    % com_instantaneous = deval(com_ode,t);
    com_instantaneous = pchip(final_times, final_solution, t);

    % Plotting positions

    instantaneous_x = com_instantaneous(1);
    instantaneous_y = com_instantaneous(2);
    instantaneous_vx = com_instantaneous(3);
    instantaneous_vy = com_instantaneous(4);

    com_position.XData = instantaneous_x;
    com_position.YData = instantaneous_y;

    com_velocity.XData = instantaneous_vx;
    com_velocity.YData = instantaneous_vy;

    % Plotting trajectories
    
    time_span = linspace(0,t,1000);  % Taking 1000 datapoints from 0 to current position along position function
    % com_from_start = deval(com_ode,time_span);
    com_from_start = pchip(final_times, final_solution, time_span);

    x_from_start = com_from_start(1,:);
    y_from_start = com_from_start(2,:);
    vx_from_start = com_from_start(3,:);
    vy_from_start = com_from_start(4,:);

    com_position_variation.XData = x_from_start;
    com_position_variation.YData = y_from_start;

    com_velocity_variation.XData = vx_from_start;
    com_velocity_variation.YData = vy_from_start;

    drawnow; % Draw plot

    t = toc*animation_speed;    % toc stops the clock in matlab; this gives time passing in seconds

end