function output = single_leg_run_event_detection(t,r,leg_length,k,g,mass,y_land,attack_angle,beta,switched,eqn1)

x = r(1);
y = r(2);

x_dot = r(3);
y_dot = r(4);

command_velocity = [x_dot;y_dot];

if switched

    if eqn1

        p = (k/mass)*(-1 + (leg_length)/(sqrt((beta - x)^2 + y^2)));
        
        x_ddot_s = p*(-beta + x);
        y_ddot_s = -g + p*y;
        
        command_acceleration = [x_ddot_s;y_ddot_s];

    elseif ~eqn1

        p = (k/mass)*(-1 + (leg_length)/(sqrt((beta - x)^2 + y^2)));
        
        x_ddot_s = p*(-beta + x);
        y_ddot_s = -g + p*y;
        
        command_acceleration = [x_ddot_s;y_ddot_s];

    end

elseif ~switched

    % Flight phase
      
    x_ddot_f = 0;
    
    y_ddot_f = -g;
    
    command_acceleration = [x_ddot_f;y_ddot_f];
    
end

output = [command_velocity;command_acceleration];

end