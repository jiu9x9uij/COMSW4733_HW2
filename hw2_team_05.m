%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%% COMS W4733 Computational Aspects of Robotics 2015 - Homework 2                                                                     %%
%%                                                                                                                                                                                                          %%
%% Team 5                                                                                                                                                                                           %%
%% Hua Tong (ht2334)                                                                                                                                                                      %%
%% Mengdi Zhang (mz2472)                                                                                                                                                            %%
%% Yilin Long (yl3179)                                                                                                                                                                      %%
%%                                                                                                                                                                                                          %%
%% README                                                                                                                                                                                        %%
%% Plotting of robot path will be painted after robot stops moving.                                                                                    %%
%% Since we cannot get position information before the robot start to move, the plot is drawn with robot's         %%
%% local coordinate system. This means, in the plot, the origin corresponds to the robot's start position,            %%
%% positive x-axis corresponds to the robot's left, and positive y-axis corresponds to the robot's front.               %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%

function  hw2_team_05(serPort)
    % Current position coordinates; Positive x-axis towards left of robot initial
    % position, positive y-axis towards front of robot initial position
    pos_x = 0;
    pos_y = 0;
    pos_theta = 0;
    % Last hit point coordinates
    q_x = pos_x;
    q_y = pos_y;
    % Step changes in distance and angle; Used to calculate positions
    d_dist = DistanceSensorRoomba(serPort);                % Reset distance difference to avoid left-over from last run
    d_theta = AngleSensorRoomba(serPort);                   % Reset angle difference to avoid left-over from last run
    % Array of positions the robot has been to; Used to plot its path
    path = [0, 0];
    
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
            sprintf('On-M-Line x = %f, y = %f, theta = %f', pos_x, pos_y, pos_theta)
            pos_theta = pos_theta + d_theta;
            pos_x = pos_x + sin(pos_theta) * d_dist;
            pos_y = pos_y + cos(pos_theta) * d_dist;
            path = [path; pos_x, pos_y];

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
            SetFwdVelRadiusRoomba(serPort, 0.3, inf);
            pause(0.1);
        end
        % Stop
        SetFwdVelRadiusRoomba(serPort, 0, inf);

        %% If goal is reached, finished
        if (target_is_reached (pos_x, pos_y, 0, 4, 0.3, 0.1))
            sprintf('I reached goal yay!')
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
        flag = false;
        no_wall_count = 0;
        while (true)
            % Set flag to true when leaving the specified threshold range after first hit
            if (~target_is_reached(pos_x, pos_y, q_x, q_y, 0.2, 0.1))   % This threshold on x must be at least twice the threshold for m-line check
                flag = true;
            end
            
            % Reached goal
            if (target_is_reached(pos_x, pos_y, 0, 4, 0.3, 0.1))
                break;
            end
            
            % Encounter m-line
            if (flag && target_is_reached(pos_x, pos_y, 0, pos_y, 0.1, 0.1))
                if (abs(pos_y - 4) < abs(q_y - 4))
                    sprintf('Encounter-M-Line, update hitpoint')
                    break;
                end
                if (target_is_reached(pos_x, pos_y, q_x, q_y, 0.2, 0.2))
                    sprintf('Encounter-M-Line, is last hitpoint')
                    break;
                end
            end

            % Read sensors
            [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);	% Read Bumpers
            WallSensor = WallSensorReadRoomba(serPort);     % Read wall sensor
            d_dist = DistanceSensorRoomba(serPort);                % Update distance difference
            d_theta = AngleSensorRoomba(serPort);                   % Update angle difference

            % Update location information
            sprintf('Wall-Following x = %f, y = %f, theta = %f, hit_x = %f, hit_y = %f', pos_x, pos_y, pos_theta, q_x, q_y)
            pos_theta = pos_theta + d_theta;
            pos_x = pos_x + sin(pos_theta) * d_dist;
            pos_y = pos_y + cos(pos_theta) * d_dist;
            path = [path; pos_x, pos_y];

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
                    % If consecutively not detecting wall, turn right more in place and reset counter
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
        end
        % Stop
        SetFwdVelRadiusRoomba(serPort, 0, inf);

        %% If goal is reached, finish
        if (target_is_reached (pos_x, pos_y, 0, 4, 0.3, 0.1))
            sprintf('I reached goal yay!')
            break;
        end

        %% If goal is unreachable, finish
        if (target_is_reached (pos_x, pos_y, q_x, q_y, 0.2, 0.3))
            sprintf('Dont u try to trick me! Im trapped.')
            break;
        end

        %% Otherwise, on m-line, turn towards goal
        if (pos_y <= 4) 
            turnAngle(serPort, 0.1, -pos_theta/pi*180);
        else
            turnAngle(serPort, 0.1, (pi-pos_theta)/pi*180);
        end
        SetFwdVelRadiusRoomba(serPort, 0, inf);
        pause(0.3);
    end
   
    %% Plot path of the robot
	plot(path(:, 1), path(:, 2));
end
    
%% Helper function to determine if distance between two points are within threshold
function  result = target_is_reached (pos_x, pos_y, tar_x, tar_y, thr_x, thr_y)
    if (abs(pos_x - tar_x) <= thr_x && abs(pos_y - tar_y) <= thr_y)
        result = true;
    else
        result = false;
    end
%     sprintf('T tar_x = %f, tar_y = %f, result = %d', tar_x, tar_y, result)
end
