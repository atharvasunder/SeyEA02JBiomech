%% Function containing ODE solver for COM position, returns no of steps for a given set of initial conditions

function no_of_steps = walking_step_counter(leg_length,k,g,mass,y_land,attack_angle,beta,t_start,t_end,t_span,initial_condition)
    
    no_of_steps = 0;    % Stores the no of strides of stable walking for a given initial condition

    % ODE45 settings
    tolerance_val = 1e-12;
    options = odeset('AbsTol',tolerance_val,'RelTol',tolerance_val,'Events', @(t, y) switchEvent(t, y, y_land, initial_condition(2)), 'Refine',10);

    switched = false;   % Flag to indicate if the system has switched
    
    final_times = [];   % Initialize arrays to store the solution
    final_solution = [];

    % Loop over time until the entire time span is covered
    while t_start < t_end
    
        com_ode = ode45(@(t,r)single_leg_run_event_detection(t,r,leg_length,k,g,mass,y_land,attack_angle,beta,switched),t_span,initial_condition,options);
        
        % disp('ode done')
    
        T = com_ode.x;  % Time values
        Y = com_ode.y;  % State values
        TE = com_ode.xe;
        YE = com_ode.ye;
        IE = com_ode.ie;   % Indices of the events that were triggered (e.g., 1 for the first event, 2 for the second event).
    
        % Store the results
        final_times = [final_times, T(1:end - 1)];    % Append time points, till end - 1 to avoid duplicates while combining arrays
        final_solution = [final_solution, Y(:, 1: end - 1)];    % Append state values
    
        % Check if an event occurred (condition translates to if is not empty)
        if ~isempty(TE)
            
            % disp("TE = ")
            % disp(TE)
    
            if IE(end) == 2 || IE(end) == 3 % Condition to end calculation when body falls or when body height crosses the initial apex height
                % fprintf('walking failed in %d step(s)',no_of_steps)
                break
            end
    
            % Update initial conditions after event
            initial_condition = Y(:,end);          % Initial condition for next phase
    
            % disp(initial_condition(2) - y_land)
    
            switched = ~switched;   % Toggle the switched state
            
            if switched == true    % We define beta if leg is initially in flight phase then reaches stance phase
                x_land = Y(1,end);   % x position of COM at which landing occurs
                beta = x_land + leg_length*cosd(attack_angle);
                no_of_steps = no_of_steps + 1;
            end
    
            if no_of_steps >= 24     % Break out of the while loop if no of steps > 24
                % disp('no of steps exceeded 24')
                break       
            end
    
            % Update the time span for the next iteration
            t_start = T(end);
            t_span = [t_start t_end];
    
        else
            t_start = T(end);   % Only for ending of t_span where there is no event invoked
    
        end
    end
end