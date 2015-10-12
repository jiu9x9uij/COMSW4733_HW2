function  hw1_team_05(serPort)
    pos_x = 0;
    pos_y = 0;
    pos_theta = 0;
    q_x = pos_x;
    q_y = pos_y;

    while (true)  
        %% Follow m-line
        BumpRight = 0;
        BumpLeft = 0;
        BumpFront = 0;
        while (true)
            % Read sensors
            [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);	% Read Bumpers
            d_dist = DistanceSensorRoomba(serPort);                % Update distance difference
            d_theta = AngleSensorRoomba(serPort);                   % Update angle difference

            % Update location information
            sprintf('M x = %f, y = %f, theta = %f', pos_x, pos_y, pos_theta)
            pos_theta = pos_theta + d_theta;
            pos_x = pos_x + sin(pos_theta) * d_dist;
            pos_y = pos_y + cos(pos_theta) * d_dist;

            % If reached goal, stop following m-line
            if (target_is_reached (pos_x, pos_y, 0, 4, 0.3, 0.1))
                break;
            end
            
            % If bump, stop following m-line
            if (BumpFront || BumpLeft || BumpRight)
                % Update hit point
                q_x = pos_x;
                q_y = pos_y;
                break;
            end
            
            % Follow m-line, i.e. go alone positive y-axis
            SetFwdVelRadiusRoomba(serPort, 0.1, inf);
            pause(0.1);
        end
        % Stop
        SetFwdVelRadiusRoomba(serPort, 0, inf);
        display(pos_x);
        display(pos_y);

        %% If goal is reached, finished
        if (target_is_reached (pos_x, pos_y, 0, 4, 0.3, 0.1))
            break;
        end
        
        %% Turn wall sensor to face wall
        if(BumpRight)
            % Right side bump into wall, turn left 45 degrees
            turnAngle(serPort, 0.1, 45);
            SetFwdVelRadiusRoomba(serPort, 0, inf);
        elseif(BumpFront)
            % Front side bump into wall, turn left 90 degrees
            turnAngle(serPort, 0.1, 90);
            SetFwdVelRadiusRoomba(serPort, 0, inf);
        else
            % Left side bump into wall, turn left 135 degrees
            turnAngle(serPort, 0.1, 135);
            SetFwdVelRadiusRoomba(serPort, 0, inf);
        end
        
        %% Follow wall
        count = 0;
        while ((~target_is_reached(pos_x, pos_y, 0, 4, 0.3, 0.1) && ~target_is_reached(pos_x, pos_y, 0, pos_y, 0.1, 0.1))...
                || (target_is_reached(pos_x, pos_y, 0, pos_y, 0.1, 0.1) && abs(pos_y - 4) >= abs(q_y - 4))...
                || count < 5)
            % Read sensors
            [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);	% Read Bumpers
            WallSensor = WallSensorReadRoomba(serPort);     % Read wall sensor
            d_dist = DistanceSensorRoomba(serPort);                % Update distance difference
            d_theta = AngleSensorRoomba(serPort);                   % Update angle difference

            % Update location information
            sprintf('W x = %f, y = %f, theta = %f, hit_x = %f, hit_y = %f', pos_x, pos_y, pos_theta, q_x, q_y)
            pos_theta = pos_theta + d_theta;
            pos_x = pos_x + sin(pos_theta) * d_dist;
            pos_y = pos_y + cos(pos_theta) * d_dist;

            %% Strategy to follow the wall
            if BumpFront
                % Front side bumped into wall, turn left 90 degrees
                turnAngle(serPort, 0.1, 90);
                SetFwdVelRadiusRoomba(serPort, 0, inf);
                no_wall_count = 0;                                          % Reset counter since not consecutively no wall anymore
            elseif (BumpRight)
                turnAngle(serPort, 0.1, 5);
                SetFwdVelRadiusRoomba(serPort, 0.05, inf);
                pause(0.2); 
                no_wall_count = 0;                                          % Reset counter since not consecutively no wall anymore
            elseif ~WallSensor
                if no_wall_count == 1
                    % If consecutively not detecting wall, turn right more in place
                    % and reset counter
                    turnAngle(serPort, 0.05, -5);
                    no_wall_count = 0;                                     % Reset counter since not consecutively no wall anymore
                else
                    % First time not detecting wall, turn right and move forward
                    turnAngle(serPort, 0.05, -5);
                    SetFwdVelRadiusRoomba(serPort, 0.05, inf);
                    pause(0.2); 
                    no_wall_count = no_wall_count + 1;         % Increase counter because consecutively no wall
                end
            else 
                SetFwdVelRadiusRoomba(serPort, 0.2, inf);
                no_wall_count = 0;                                         % Reset counter since not consecutively no wall anymore 
            end

            pause(0.1); 
            count = count + 1;
        end
        % Stop
        SetFwdVelRadiusRoomba(serPort, 0, inf);
        display(pos_x);
        display(pos_y);

        %% If goal is reached, 
        if (target_is_reached (pos_x, pos_y, 0, 4, 0.3, 0.1))
            break;
        end

        %% Goal is unreachable
        if (target_is_reached (pos_x, pos_y, q_x, q_y, 0.1, 0.2))
            break;
        end

        %% On m-line, turn towards positive y-asix
        if (pos_y <= 4) 
            turnAngle(serPort, 0.1, -pos_theta/pi*180);
        else
            turnAngle(serPort, 0.1, (pi-pos_theta)/pi*180);
        end
        SetFwdVelRadiusRoomba(serPort, 0, inf);
        pause(0.3);
    end
end
    

function  result = target_is_reached (pos_x, pos_y, tar_x, tar_y, thr_x, thr_y)
    if (abs(pos_x - tar_x) <= thr_x && abs(pos_y - tar_y) <= thr_y)
        result = true;
    else
        result = false;
    end
    sprintf('T tar_x = %f, tar_y = %f, result = %d', tar_x, tar_y, result)
end
