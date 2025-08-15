%% Function that takes an initial condition, runs 1 step and returns the apex height at end of that step

function [y_2,final_solution,takeoff_coordinates,landing_coordinates] = poincare_function(leg_length,k,g,mass,y_land,attack_angle,beta,t_start,t_end,t_span,initial_condition)

    if initial_condition(2) > y_land     
        switched = false;   % Flag to indicate if the system has switched
    else
        switched = true;
        % beta = initial_condition(1) + leg_length*cosd(attack_angle);  %
        % Dont update beta because we are studying howq the system behaves
        % with different iniitial conditions. So system should stay as is
    end

    eqn1 = true;     % Variable to define which equation of motion is used during stance phase

    y_2 = 0;    % Storing it as a dummy variable because I am assigning the actual value to y_2 in an if condition

    final_times = [];   % Initialize arrays to store the solution
    final_solution = [];

    takeoff_coordinates = [];
    landing_coordinates = [];

    % Loop over time until the entire time span is covered
    while t_start < t_end

        % ODE45 settings
        tolerance_val = 1e-12;
        options = odeset('AbsTol',tolerance_val,'RelTol',tolerance_val, 'MaxStep', 0.01 ,'Events', @(t, y) switchEvent(t, y, y_land, beta, leg_length, switched));
        
        com_ode = ode45(@(t,r)single_leg_run_event_detection(t,r,leg_length,k,g,mass,y_land,attack_angle,beta,switched,eqn1),t_span,initial_condition,options);
        
        % disp('ode done')
    
        T = com_ode.x;  % Time values
        Y = com_ode.y;  % State values
        TE = com_ode.xe;
        YE = com_ode.ye;
        IE = com_ode.ie;   % Indices of the events that were triggered (e.g., 1 for the first event, 2 for the second event).

        % Store the results
        final_times = [final_times, T(2:end)];    % Append time points, till end - 1 to avoid duplicates while combining arrays
        final_solution = [final_solution, Y(:, 2: end)];    % Append state values
    
        % Check if an event occurred (condition translates to if is not empty)
        if ~isempty(TE)
            
            % disp("YE = ")
            % disp(YE(:,end))
             
            % disp("end t per ode solve =")
            % disp(YE(:,end))

            if IE(end) == 2 % Change from stance to flight when spring length becomes equal to leg length
                x_takeoff = Y(1,end);   % x position of COM at which takeoff occurs
                y_takeoff = Y(2,end);   % y position of COM at which takeoff occurs
                
                takeoff_coordinates = [x_takeoff,y_takeoff];

                switched = ~switched;
            end

            if IE(end) == 4 % When vy = 0 and com is at apex
                y_2 = Y(2, end);    % Store latest y value as the y(i+1) after reaching the apex
                % disp(Y(4, end))
                break
            end
    
            % Update initial conditions after event
            initial_condition = Y(:,end);          % Initial condition for next phase
    
            % disp(initial_condition(2) - y_land)
            if IE(end) == 1
                switched = ~switched;   % Toggle the switched state
                y_land = Y(2,end);   % y position of COM at which landing occurs
                x_land = Y(1,end);   % x position of COM at which landing occurs
                landing_coordinates = [x_land,y_land];
                beta = x_land + leg_length*cosd(attack_angle);
                % eqn1 = ~eqn1;
            end

            % if IE(end) == 3
            % 
            %     eqn1 = ~eqn1;
            % 
            % end

            % Update the time span for the next iteration
            t_start = T(end);
            t_span = [t_start t_end];
    
        else
            t_start = T(end);   % Only for ending of t_span where there is no event invoked
    
        end
    end
end