%%  Loop to get number of steps for all initial conditions

% Constant initial velocity, varying k and attack angle

% Define a grid of attack angle and k values
attack_angle = linspace(40, 80, 100);
k = linspace(0, 50000, 200);

% Initialize a matrix to store the results
output_1 = zeros(length(k), length(attack_angle));

% Loop through all combinations of attack_angle and k
for i = 1:length(k)
    for j = 1:length(attack_angle)
        
        % Get the current values of attack_angle and k
        current_attack_angle = attack_angle(j);
        current_k = k(i);
     
        y_land = leg_length*sind(current_attack_angle);  % Height at which leg lands and leaves the ground (beginning and end of stance phase)

        % Compute the function
        output_1(i, j) = walking_step_counter(leg_length,current_k,g,mass,y_land,current_attack_angle,beta,t_start,t_end,t_span,initial_condition); 

    end
end

% Constant attack angle, varying initial velocity x and k

% Define a grid of Vx and k values
initial_velocity_x = linspace(1, 11, 50);
initial_velocity_y = zeros(size(initial_velocity_x));
k = linspace(0, 50000, 200);

% Initialize a matrix to store the results
output_2 = zeros(length(initial_velocity_x), length(k));

% Loop through all combinations of attack_angle and k
for i = 1:length(initial_velocity_x)
    for j = 1:length(k)

        % Get the current values of attack_angle and k
        current_initial_vel_x = initial_velocity_x(i);
        current_initial_vel_y = initial_velocity_y(i);

        initial_condition = [start_x;start_y;current_initial_vel_x;current_initial_vel_y];

        current_k = k(j);

        % Compute the function
        output_2(i, j) = walking_step_counter(leg_length,current_k,g,mass,og_y_land,og_attack_angle,beta,t_start,t_end,t_span,initial_condition); 

    end
end

% Constant k, varying initial velocity x and attack angle

% Define a grid of attack angle and Vx values
initial_velocity_x = linspace(1, 11, 50);
initial_velocity_y = zeros(size(initial_velocity_x));
attack_angle = linspace(40, 80, 100);

% Initialize a matrix to store the results
output_3 = zeros(length(initial_velocity_x), length(attack_angle));

% Loop through all combinations of attack_angle and k
for i = 1:length(initial_velocity_x)
    for j = 1:length(attack_angle)

        % Get the current values of attack_angle and k
        current_initial_vel_x = initial_velocity_x(i);
        current_initial_vel_y = initial_velocity_y(i);

        initial_condition = [start_x;start_y;current_initial_vel_x;current_initial_vel_y];

        current_attack_angle = attack_angle(j);

        y_land = leg_length*sind(current_attack_angle);  % Height at which leg lands and leaves the ground (beginning and end of stance phase)

        % Compute the function
        output_3(i, j) = walking_step_counter(leg_length,og_k,g,mass,y_land,current_attack_angle,beta,t_start,t_end,t_span,initial_condition); 

    end
end