function [value, isterminal, direction] = switchEvent(t, y, y_land, beta, leg_length, switched)
    % disp('event detected')
    % disp("height = ")
    % disp(y(2))
    % disp("y_land = ")
    % disp(y_land)
    % Event triggered when y crosses y_land
    
    % disp('beta')
    % disp(beta)

    x_com = y(1);
    y_com = y(2);

    if switched == false
        spring_length = leg_length;
    elseif switched == true
        spring_length = sqrt((x_com - beta)^2 + y_com^2);   % Gives the instantaneous length of the leg/spring
    end

    value(1) = y(2) - y_land;   % When y(2) - y_land changes sign, trigger event
    value(2) = spring_length - leg_length;  % When spring length = leg length, end stance phase
    % value(2) = y(2);    % Falling event
    % value(3) = y(1) - start_y;      % Check if com goes above pivot point exactly
    value(4) = y(4);    % The moment vy becomes = 0, trigger event

    % disp('value')
    % disp(value)

    isterminal(1) = 1;       % Stop the integration when event occurs
    isterminal(2) = 1;
    % isterminal(3) = 1;
    isterminal(4) = 1;

    direction(1) = -1;        % Detects zero-crossings only when the event function is decreasing, i.e., when it crosses zero from positive to negative.
    direction(2) = 1;
    % direction(3) = 0;     
    direction(4) = -1;  % Detects zero-crossings only when the event function is decreasing, i.e., when it crosses zero from positive to negative.

end