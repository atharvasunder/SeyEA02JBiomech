clear;
clc;
close all;
%% System Parameters

leg_length = 1;     % In meters
g = 9.81;   % m/s^2
og_k = 20000;     % N/m
mass = 80;   % Mass of runner, kg

%% Initial conditions

og_attack_angle = 68;  % In degrees
start_x = 0;
start_y = 1;
start_vx = 5;   % m/s
start_vy = 0;

system_energy = 0.5*mass*start_vx^2 + mass*g*1;   % Total energy of system

% start_vx = sqrt((2/mass)*(system_energy - mass*g*start_y))

equivalent_height = system_energy/(mass*g)

% global beta;
beta = 0;

initial_condition = [start_x;start_y;start_vx;start_vy];

%% Mode switching conditions from stance to swing

og_y_land = leg_length*sind(og_attack_angle)  % Height at which leg lands and leaves the ground (beginning and end of stance phase)

%% Defining simulation duration parameters

animation_speed = 0.1;    % Animation speed

t_start = 0;
t_end = 4;

t_span = [t_start,t_end];     % Inputted into ode45, the span over which the numerical solution is computed

%% Calculate number of steps for different initial conditions

% step_calculator

%% Calculate the height of next step for different initial conditions

% next_step_height_calculator

%% ODE Solution

no_of_steps = 0;    % Stores the no of strides of stable walking for a given initial condition

if start_y > og_y_land      % switched = false during flight phase
    switched = false;   % Flag to indicate if the system has switched
else
    switched = true;
end

eqn1 = true;     % Variable to define which equation of motion is used during stance phase

final_times = [];   % Initialize arrays to store the solution
final_solution = [];
PE = [];       % Energy terms
KE = [];

spring_length = leg_length;     % Initialize spring length to be same as leg length for initial flight phase
key = false;    % Used to define the springs length based on phase

x_land = 0;     % Just for no error the first time the switchevent is called

% Loop over time until the entire time span is covered
while t_start < t_end

    % ODE45 settings
    tolerance_val = 1e-12;
    options = odeset('AbsTol',tolerance_val,'RelTol',tolerance_val,'Events', @(t, y) switchEvent(t, y, og_y_land, beta, leg_length, switched));

    com_ode = ode45(@(t,r)single_leg_run_event_detection(t,r,leg_length,og_k,g,mass,og_y_land,og_attack_angle,beta,switched,eqn1),t_span,initial_condition,options);

    % disp('ode done')

    T = com_ode.x;  % Time values
    Y = com_ode.y;  % State values
    TE = com_ode.xe;    % Time of event
    YE = com_ode.ye;
    IE = com_ode.ie;   % Indices of the events that were triggered (e.g., 1 for the first event, 2 for the second event)

    % Store the results
    final_times = [final_times, T(2:end)];    % Append time points, till end - 1 to avoid duplicates while combining arrays
    final_solution = [final_solution, Y(:, 2: end)];    % Append state values

    temp_y = Y(:, 1: end - 1);  % Stores all the states solved for in this ode run

    for i = 1:size(temp_y,2)    % loop through all columns

        x_com = temp_y(1,i);
        y_com = temp_y(2,i);
        vx_com = temp_y(3,i);
        vy_com = temp_y(4,i);

        if key == false
            spring_length = leg_length;
        elseif key == true
            spring_length = sqrt((x_com - beta)^2 + (y_com)^2);
        end

        state_PE = 0.5*og_k*(spring_length - leg_length)^2 + mass*g*y_com;
        state_KE = 0.5*mass*(vx_com^2 + vy_com^2);

        KE = [KE , state_KE];
        PE = [PE , state_PE];

    end

    % Check if an event occurred (condition translates to if is not empty)
    if ~isempty(TE)

        % disp("IE = ")
        % disp(IE)

        if IE(end) == 2 % Change from stance to flight when spring length becomes equal to leg length
            %  verified
            disp('state switched from stance to flight')
            disp('beta = ')
            disp(beta)

            check = ((Y(1,end) - beta)^2 + (Y(2,end) - 0)^2)^0.5
            switched = ~switched;
            key = ~key;
        end

        % if IE(end) == 4 % When vy = 0 and com is at apex
        %     disp('at apex')
        %     disp(Y(:,end))
        %     y_2 = Y(2, end);    % Store latest y value as the y(i+1) after reaching the apex
        % 
        %     x_apex = Y(1,end);
        %     y_apex = Y(2,end);
        % 
        %     % spring_length = ((x_land - x_apex)^2 + (y_land - 0)^2)^0.5
        %     % check = leg_length
        % 
        %     break
        % end

        % disp('not at apex')

        % disp(initial_condition(2) - y_land)

        if IE(end) == 1
            disp('state switched from flight to stance')
            switched = ~switched;   % Toggle the switched state
            key = ~key;
            disp(Y(:, end))

            x_land = Y(1,end);   % x position of COM at which landing occurs
            y_land = Y(2,end);
            % disp("beta updated")
            beta = x_land + leg_length*cosd(og_attack_angle);
            disp('beta = ')
            disp(beta)
            % disp(leg_length*sind(og_attack_angle) - Y(2,end)) 
            % beta is very accurate
            % no_of_steps = no_of_steps + 1;
            % disp(Y(1,end) - beta)

        end

        % disp(Y(1,end) - beta)

        % if IE(end) == 3
        % 
        %     eqn1 = ~eqn1;
        % 
        %     disp('x')
        %     disp(Y(1,end))
        %     disp('beta')
        %     disp(beta)
        %     disp('x - beta')
        %     disp(Y(1,end) - beta)
        %     disp('y')
        %     disp(Y(:,end))
        % 
        % end

        % if no_of_steps >= 24     % Break out of the while loop if no of steps > 24
        %     % disp('no of steps exceeded 24')
        %     break       
        % end

        % Update initial conditions after event
        initial_condition = Y(:,end);          % Initial condition for next phase

        % Update the time span for the next iteration
        t_start = T(end);
        t_span = [t_start t_end];

    else
        t_start = t_end;   % Only for ending of t_span where there is no event invoked

    end
end
%% Running the animation/visualization

animation
% visualization
% visualization_poincare