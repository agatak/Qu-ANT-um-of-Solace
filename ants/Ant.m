classdef Ant
    
    properties
        localVector         % stores local vectors visible to ant
        globalVector        % stores ants global vector (in ant-tied coordinate system)
        lightVector         % stores current e-vector
        feederFound         % stores location of found feeders
        feeder_globalVector % stores global vector pointing to feeder in ant coordinate system
        feeder_meanVector   % stores mean vector pointing to feeder in global coordinate system
        meanVector          % contains mean vector, calculated based on global, local and e-vector
        mode                % defines if ant is foraging or homing (1 - foraging, 2 - homing)
        direction           % stores direction in a form 1 - 9 (dir_lookup) of a previous movement
        sightRange          % defines how good is ant sight
        fieldOfVision       % stores objects in area that the ant can see
        trail               % stores current trail of ant
        pastTrails          % stores history of runs
        nestDist            % stores the value of distance at which ant is in relatino to nest
        position            % stores ant position on map, not used for navigation
        dir_lookup={[1,1],[0,1],[-1,1],[-1,0],[-1,-1],[0,-1],[1,-1],[1,0]} % vector containing all possible directions of movement (starting from right, then right up and so on in counter clockwise manner)
        obstacleEncountered % flag, marking if ant tried to run into obstacle
        intended_dir        % in case ant encounteres obstacle, stores value of intended direction, for the whole time obstacle is blocking the way
        walkType            % informas what type of movement ant is engaded in
        e_ant_angle         % ant_angle of ant body with respect to lightVector (e-vector)
        rot_error           % arbitrary assumed rotation error, which is multiplied by rotation in global vector calculation
        spiral              % used for ant looking for nest when can't find it when homing
        spiral_turn         % as above
        spiral_step         % as above
    end
    
    methods
        %% Initialize/create ant
        function obj=Ant(mode,world,sightRange,position,rotationerror)
            
            
            obj.mode=mode;              % 1=foraging, 0=returning to nest
            obj.sightRange=sightRange;
            obj.feederFound=false;      % feeder is not known at the beginning
            obj.globalVector=[0 0];
            obj.trail{1}=position;
            obj.position=position;
            obj.nestDist=norm(obj.position-world.nest{1});
            obj.obstacleEncountered=false;
            obj.rot_error=rotationerror;
            obj.spiral=1;
            obj.spiral_turn=1;
            obj.spiral_step=1;
            
        end
        
        %% Ant sight- define what objects ant see in its field of view
        function obj=antSight(obj,world)
            
            obj.fieldOfVision=[];
            counter=numel(world.food);                          % check how many of object type X exists in the world
            
            m=1;                                                % reset counter
            obj.fieldOfVision{1}=[];                            % preallocate (in case none of these is in vision, still there's an empty cell)
            for n=1:counter                                     % for each existing object of type X
                distance=world.food{n}-obj.position;            % check how far it is from ant
                if norm(distance)<=obj.sightRange               % and if it is within sight range
                    obj.fieldOfVision{1}{m}=distance;               % store in repsective cell in obj.fieldOfVision variable
                    m=m+1;                                          % increase counter
                end
            end
            
            counter=numel(world.nest);                          % REPETITION OF THE ABOVE
            
            m=1;
            obj.fieldOfVision{2}=[];
            for n=1:counter
                distance=world.nest{n}-obj.position;
                if norm(distance)<=1/3*obj.sightRange           % HARDER TO SEE NEST than anything else
                    obj.fieldOfVision{2}{m}=distance;
                    m=m+1;
                end
            end
            
            counter=numel(world.landmarks)/2;                   % REPETITION OF THE ABOVE BUT!!!!
            
            m=1;
            obj.fieldOfVision{3}=[];
            obj.localVector=[];                                 % ALSO STORES LOCAL VECTORS ACCOMPANYING EACH LANDMARK
            for n=1:counter
                distance=world.landmarks{1,n}-obj.position;
                temp_localVector=world.landmarks{2,n};
                if norm(distance)<=obj.sightRange               % only if it lies within ant's sight range ofcourse
                    obj.fieldOfVision{3}{m}=distance;
                    obj.localVector{m}=temp_localVector;
                    m=m+1;
                end
            end
            
            counter=numel(world.obstacles);
            
            m=1;
            obj.fieldOfVision{4}=[];
            for n=1:counter
                distance=world.obstacles{n}-obj.position;
                if norm(distance)<=obj.sightRange
                    obj.fieldOfVision{4}{m}=distance;
                    m=m+1;
                end
            end
            
        end
        
        %% Store and work with local vector (memory of landmarks)
        function obj=landmark(obj)
            loc_vec_x=0;
            loc_vec_y=0;
            loc_vec_counter=numel(obj.localVector);
            
            for n=1:loc_vec_counter
                obj.localVector{n}=obj.localVector{n}+obj.fieldOfVision{3}{n};      % sum local vector and vector pointing from ant to landmark to get vector pointing from ant to nest
                loc_vec_x=obj.localVector{n}(1)+loc_vec_x;
                loc_vec_y=obj.localVector{n}(2)+loc_vec_y;
            end
            
            % average all seen local vectors
            loc_vec_x=loc_vec_x/loc_vec_counter;
            loc_vec_y=loc_vec_y/loc_vec_counter;
            obj.localVector=[loc_vec_x, loc_vec_y];
            
        end
        
        %% Path integration
        function obj=pathIntegration(obj)
            
            ant_orientation=obj.dir_lookup{obj.direction};
            previous_e_ant_angle=obj.e_ant_angle;
            obj.e_ant_angle=radtodeg(atan2(ant_orientation(1)*obj.lightVector{1}(2)-ant_orientation(2)*obj.lightVector{1}(1),ant_orientation(1)*ant_orientation(2)+obj.lightVector{1}(1)*obj.lightVector{1}(2)));  % get ant_angle between two vectors 0-2pi http://www.mathworks.com/matlabcentral/newsreader/view_thread/151925
            
            if obj.e_ant_angle<0                                % THIS FUNCTION GIVES WRONG RESULTS FOR NEGATIVE ANGLES, this compensates
                obj.e_ant_angle=-(180+obj.e_ant_angle);
            elseif obj.lightVector{1}+ant_orientation==[0 0]    % for some reason method cannot see 180 degrees - this compensates for that!
                obj.e_ant_angle=180;
            end
            
            rotation=-obj.e_ant_angle+previous_e_ant_angle;     % rotation = rotation rate, because each rotation is done in one unit time
            rotation=rotation*obj.rot_error;                    % include rotational error  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            velocity=norm(ant_orientation);                     % ant velocity is equal to it's previous step divided by one time unit, as each step is done in one time unit
            obj.globalVector=obj.globalVector*[cosd(rotation), -sind(rotation); sind(rotation), cosd(rotation)];    % rotate global vector to a new frame, to which ant rotated
            obj.globalVector(2)=obj.globalVector(2)-velocity;   % add ant translation in y direction (ant always moves in Y direction in its frame of reference)
            
            if obj.feederFound                                  % in case location of feeder was found, perform path integration on the way back to nest, and store global vector pointing to feeder
                obj.feeder_globalVector=obj.feeder_globalVector*[cosd(rotation), -sind(rotation); sind(rotation), cosd(rotation)];
                obj.feeder_globalVector(2)=obj.feeder_globalVector(2)-velocity;
            end
            
        end
        
        %% Obtain vector from position of sun / light polarization
        function obj=lightCompass(obj)
            % define simplified polarizationCompass in terms of one of the
            % 8 directions present in ant's world
            obj.lightVector{1}=obj.dir_lookup{randi([1,8])};
            ant_initial_vec=[0 1];
            obj.e_ant_angle=radtodeg(atan2(ant_initial_vec(1)*obj.lightVector{1}(2)-ant_initial_vec(2)*obj.lightVector{1}(1),ant_initial_vec(1)*ant_initial_vec(2)+obj.lightVector{1}(1)*obj.lightVector{1}(2)));    % at first ant is oriented as the global refence system
            if obj.e_ant_angle<0                                   % THIS FUNCTION GIVES WRONG RESULTS FOR NEGATIVE ANGLES, this compensates
                obj.e_ant_angle=-(180+obj.e_ant_angle);
            elseif obj.lightVector{1}+ant_initial_vec==[0 0]       % for some reason method cannot see 180 degrees - this compensates for that!
                obj.e_ant_angle=180;
            end
        end
        
        %% Navigate to feeder
        function obj=feederVec(obj)
            ant_dir=obj.dir_lookup{obj.direction};
            global_vec=[0 1];
            theta=radtodeg(atan2(ant_dir(1)*global_vec(2)-ant_dir(2)*global_vec(1),ant_dir(1)*ant_dir(2)+global_vec(1)*global_vec(2)));  % get ant_angle between two vectors http://www.mathworks.com/matlabcentral/newsreader/view_thread/151925
            if theta<0                             % THIS FUNCTION GIVES WRONG RESULTS FOR NEGATIVE ANGLES, this compensates
                theta=-(180+theta);
            elseif ant_dir+global_vec==[0 0]       % for some reason method cannot see 180 degrees - this compensates for that! (sum of vectors = 0 0 means vectors have 180 degrees difference)
                theta=180;
            end
            
            vector_feederCoord=obj.feeder_globalVector*[cosd(theta), -sind(theta); sind(theta), cosd(theta)];               % transform ant's global vector to global coordinates
            obj.feeder_meanVector=vector_feederCoord;
        end
        
        %% Use all available tracking mechanisms to devise a walking direction to nest
        function obj=meanVec(obj)
            ant_dir=obj.dir_lookup{obj.direction};
            global_vec=[0 1];
            theta=radtodeg(atan2(ant_dir(1)*global_vec(2)-ant_dir(2)*global_vec(1),ant_dir(1)*ant_dir(2)+global_vec(1)*global_vec(2)));  % get ant_angle between two vectors http://www.mathworks.com/matlabcentral/newsreader/view_thread/151925
            if theta<0                             % THIS FUNCTION GIVES WRONG RESULTS FOR NEGATIVE ANGLES, this compensates
                theta=-(180+theta);
            elseif ant_dir+global_vec==[0 0]       % for some reason method cannot see 180 degrees - this compensates for that! (sum of vectors = 0 0 means vectors have 180 degrees difference)
                theta=180;
            end
            
            vector_globCoord=obj.globalVector*[cosd(theta), -sind(theta); sind(theta), cosd(theta)];               % transform ant's global vector to global coordinates
            
            if ~isempty(obj.localVector) && obj.walkType==6 % if there's a local vector in range, calculate mean of it and global vector as a direction
                
                obj.meanVector=[(vector_globCoord(1)+obj.localVector(1))/2,(vector_globCoord(2)+obj.localVector(2))/2];
                obj.globalVector=obj.meanVector*[cosd(-theta), -sind(-theta); sind(-theta), cosd(-theta)]; % correct obj.globalVector based on landmark
                
            else
                obj.meanVector=vector_globCoord;
            end
            
            if norm(vector_globCoord)<1 && obj.mode==2
                obj.mode=3;
            end
        end
        %%  Walk
        function obj=walk(obj,world) %define walking direction, depending on ant goal (homing / foraging) and world (seeing objects, walking into objects)
            
            % invoke antSight function to let the ant "look around"
            obj=antSight(obj,world);
            
            
            % FORAGING MODE
            % ----------------------------------------------------------------------------------------------------------------------------------
            if obj.mode==1
                
                move_vector=[];
                % If this is the first step, pick direction at random
                if isempty(obj.direction)  && ~obj.obstacleEncountered && ~obj.feederFound
                    disp('random first walk')
                    %obj.direction=randi([1,8]);
                    obj.direction=1;
                    obj.walkType=1;                                               % walkType equal to 1 means ant is foraging and performing first random stemp
                    
                    % If no food is seen, pick a direction at random,
                    % chosen from a normal distribution based on previous
                    % direction, provided there's no obstacle in the way
                elseif ~obj.obstacleEncountered && ~obj.feederFound && isempty(obj.fieldOfVision{1})
                    disp('random walk')
                    obj.walkType=3;                                                % walkType equal to 3 means ant is foraging and not seeing any food
                    randDirection=round(normrnd(obj.direction,0.1+obj.nestDist/world.size/1.5));
                    while randDirection>8 || randDirection<1
                        randDirection=round(normrnd(obj.direction,0.1+obj.nestDist/world.size/1.5));
                    end
                    obj.direction=randDirection;
                    
                    % If the ant can "see" any food all calcualations are
                    % overwriten and the move direction points directly
                    % towards the feeder
                elseif ~isempty(obj.fieldOfVision{1}) && ~obj.obstacleEncountered
                    disp('food in range')
                    obj.walkType=2;                                               % walkType equal to 2 means ant is foraging and seeing food
                    
                    food_in_sightAmount=numel(obj.fieldOfVision{1});              % how many food items ant can see
                    for n=1:food_in_sightAmount                                   % for every seen food item
                        dist_to_seen_food(n)=norm(obj.fieldOfVision{1}{n});       % calculate distance to ant
                    end
                    
                    [~, closest_food]=min(dist_to_seen_food);                     % find indices of closest food in fieldOfVision matrix
                    move_vector=obj.fieldOfVision{1}{closest_food};               % define vector pointing to closesd food item seen by ant
                    
                    % in case ant knowns position of any feeder from a
                    % previous run, navigate to that feeder using
                    % food_globalVector
                elseif obj.feederFound && ~obj.obstacleEncountered && isempty(obj.fieldOfVision{1})
                    disp('going back to feeder')
                    obj.walkType=2;                                               % is equal to 2 because, behaves similarly to when food is seen
                    obj=feederVec(obj);
                    move_vector=obj.feeder_meanVector;
                    
                end
                
                % evaluate move_vector, if exists (if ant sees food, it
                % WILL exist for sure, as walkType=2 will be invoked
                if ~isempty(move_vector)
                    if norm(move_vector)<1                                       % if ant arrived at FOOD location
                        obj.mode=2;                                                % change mode to homing (NOT ASSIGN DIRECION, HOMIN WILL)
                        obj.feederFound=true;
                        obj.feeder_globalVector=[0 0];                               % reset global vector pointing to feeder
                    else
                        [ant_angle,~] = cart2pol(move_vector(1),move_vector(2));   % based on vector, get polar coordinates pointing to closest food item direction
                        ant_angle=rad2deg(ant_angle);                             % convert radians to degrees
                        if ant_angle<0
                            ant_angle=360+ant_angle;                              % convert negative ant_angles to positive ones (0-360 instead of 0-180 and 0- -180)
                        elseif ant_angle==0                                       % prevent confusion when ant_angle = 0
                            ant_angle=1;
                        end
                        dir=round(ant_angle/45);
                        if dir==0                                                  % we have 8 directions only so dir 0 equals dir 8
                            dir=8;
                        end
                        
                        obj.direction=dir;                                         % store the new direction
                    end
                end
                
                
                
            end
            
            % HOMING MODE
            % --------------------------------------------------------------------------------------------------------------------------------
            if obj.mode==2
                
                % If the ant can "see" the nest all calcualations are
                % overwriten and the move direction
                % poinobj=antSight(obj,world)ts directly towards the
                % feeder.
                
                if ~obj.obstacleEncountered && ~isempty(obj.fieldOfVision{2})
                    disp('homin seeing nest')
                    obj.walkType=4;                                               % walkType equal to 4 means ant is homing with nest in range of sight
                    move_vector=world.nest{1}-obj.position;                       % define vector pointing to closesd food item seen by ant
                    
                elseif ~obj.obstacleEncountered && isempty(obj.localVector) && isempty(obj.fieldOfVision{2}) % use navigational tools for homing
                    disp('homin')
                    obj=meanVec(obj);
                    move_vector=obj.meanVector;
                    obj.walkType=5;                                               % walkType equal to 5 means ant is homing
                    
                elseif ~obj.obstacleEncountered && ~isempty(obj.localVector) && isempty(obj.fieldOfVision{2})
                    disp('homin seeing landmark')
                    obj.walkType=6;                                               % homing and seeing local vector
                    obj=landmark(obj);
                    obj=meanVec(obj);
                    move_vector=obj.meanVector;
                end
                
                % calculate direction movement based on defined move_vector
                if ~obj.obstacleEncountered
                    if norm(move_vector)==0           % if arrived to the NEST
                        obj.globalVector=[0 0];        % clear error in PI
                        obj.mode=1;                    % change mode to foraging
                        
                        if isempty(obj.pastTrails)
                            obj.pastTrails{1}=obj.trail;     % store trail of this run in history of trails, for the first run
                        else
                            obj.pastTrails{end}=obj.trail;    % store trail of this run in history of trails
                        end
                        
                        obj.trail=[];                  % clear current trail
                        obj.spiral=1;                  % reset spiral controls
                        obj.spiral_turn=1;
                        obj.spiral_step=1;
                        
                        % start going back to feeder (same code as in mode
                        % 1, when ant is going back to feeder)
                        disp('going back to feeder')
                        obj.walkType=2;                % is equal to 2 because, behaves similarly to when food is seen
                        obj=feederVec(obj);
                        move_vector=obj.feeder_meanVector;
                        
                        
                    end
                    
                    [ant_angle,~] = cart2pol(move_vector(1),move_vector(2));      % based on vector, get polar coordinates pointing to closest food item direction
                    ant_angle=rad2deg(ant_angle);                                 % convert radians to degrees
                    
                    if ant_angle<0
                        ant_angle=360+ant_angle;                                  % convert negative ant_angles to positive ones (0-360 instead of 0-180 and 0- -180)
                    elseif ant_angle==0                                           % prevent confusion when ant_angle = 0
                        ant_angle=1;
                    end
                    
                    dir=round(ant_angle/45);
                    if dir==0                                                 % we have 8 directions only so dir 0 equals dir 8
                        dir=8;
                    end
                    obj.direction=dir;
                    
                end
                
                
            end
            
            % CANT FIND NEST MODE - SPIRALING
            % --------------------------------------------------------------------------------------------------------
            if obj.mode==3 && ~obj.obstacleEncountered
                disp('cant find nest')
                obj.walkType=7;
                obj.direction=obj.spiral_turn;                   % go in the direction indicated by spiral_turn (1 for first move)
                
                if obj.spiral_step<obj.spiral                    % as long as you walked less then spiral moves (1 for 1st, 2 for 2nd and so on) in spiral_turn direction, continue in this one
                    obj.spiral_step=obj.spiral_step+1;
                else
                    obj.spiral_turn=obj.spiral_turn+1;           % if you walked amount of steps in spiral_turn direction equal to number of spiral, change direction
                    obj.spiral_step=1;                           % reset step counter, as new direction was chosen
                    if obj.spiral_turn==5                        % when half of turns are done, increase spiral counter - PROVIDES UNIFORM SPIRAL for some reason
                        obj.spiral=obj.spiral+1;
                    end
                end
                
                if obj.spiral_turn==9                            % only 8 directions available, so if you reached last one
                    obj.spiral=obj.spiral+1;                      % all of turns are done, increase spiral counter - PROVIDES UNIFORM SPIRAL for some reason
                    obj.spiral_turn=1;                            % and reset directions
                end
                
                if ~isempty(obj.fieldOfVision{2})                % if nest is found, revert to mode 2
                    obj.mode=2;
                end
            end
            
            % BASED ON CHOSEN DIRECTION, ASSIGN NEW FIELD TO WHICH ANT
            % SHALL GO
            chosenNew_position=obj.dir_lookup{obj.direction};
            
            
            % OBSTACLE AVOIDING CHECK
            % ------------------------------------------------------------------------------------------------------------------------
            if ~isempty(obj.fieldOfVision{4})                                                                       % if ant can see any obstacles in it's field of vision
                obstacle_in_the_way=find(cellfun(@(x) isequal(x,chosenNew_position), obj.fieldOfVision{4}), 1);     % check if there's an obstacle in the chosen field
                
                while ~isempty(obstacle_in_the_way)                                                                 % while there is, choose a new direction (to the left or to the right of the chosen one)
                    
                    disp('active avoiding obstacles');
                    
                    if obj.walkType==2 || obj.walkType==4  || obj.walkType==5 || obj.walkType==6 || obj.walkType==7 % engage obstacle avoiding in case ant has a motiviation to go a certain way (to food / to nest)
                        disp('stored dir')
                        obj.obstacleEncountered=true;           % MARK THE FLAG
                        obj.intended_dir=obj.direction;         % STORE INTENDED DIRECTION
                    end
                    
                    
                    neighbouringObs=obj.fieldOfVision{4}(find(cellfun(@(x) norm(x), obj.fieldOfVision{4})<2));  % check if neighbouring cells contain obstacle
                    
                    for n=1:numel(neighbouringObs)
                        forbidden_dir(n)=(find(cellfun(@(x) isequal(x,neighbouringObs{n}), obj.dir_lookup))); 	% list forbidden directions (i.e. those having obstacles)
                    end
                    
                    forbidden_dir=sort(unique(forbidden_dir));                                                  % sort forbidden directions and remove duplplicates if present
                    
                    if ismember(8,forbidden_dir) && ismember(1,forbidden_dir)                                   % if dir 1 and dir 8 are both forbidden, then, ant must used allowed dir instead
                        counter=1;
                        for n=1:8
                            if ~ismember(n,forbidden_dir)           % check if n belongs to forbidden_dir
                                allowed_dir(counter)=n;             % store are allowed directinos (empty fields around ant)
                                counter=counter+1;                  % raise counter
                            end
                        end
                        way(1)=allowed_dir(1);                      % first possible way
                        way(2)=allowed_dir(end);                    % second possible way
                    else
                        way(1)=forbidden_dir(1)-1;                  % first possible way is one lower than lowest value (e.g. if there's an obstacle at directions from 7 to 8 then one can go to dir 6)
                        way(2)=forbidden_dir(end)+1;                % second possible way is one higher than highest value (e.g. if there's an obstacle at directions from 7 to 8 then one can go to dir 9)
                    end
                    
                    
                    if way(1)==0                                    % only 8 possible directions so 0 = 8
                        way(1)=8;
                    end
                    if way(2)==9                                    % only 8 possible directions so 9 = 1
                        way(2)=1;
                    end
                    
                    choice=randi([1,2]);                            % choose between two directions closest to the obstacle - track it's boundary
                    chosenNew_position=obj.dir_lookup{way(choice)};
                    obj.direction=way(choice);                      % update ant direction (body orientation)
                    obstacle_in_the_way=find(cellfun(@(x) isequal(x,chosenNew_position), obj.fieldOfVision{4}), 1); % check again for obstacle in the new chosen field
                end
                % if the obstacle is not blocking the way, this does
                % nothing
            end
            
            
            % IF AN OBSTACLE WAS HIT, obj.obstacleEncountered FLAG IS
            % CHECKED, THEN ONLY THE PART OF CODE BELOW IS EXECUTED which
            % results in ant keeping the direction along obstacle UNLESS
            % the direction that was intended when at first found obstacle
            % is clear in that case, go the intended direction and CLEAR
            % THE obj.obstacleEncountered FLAG
            if obj.obstacleEncountered                                      % if ant hit an obstacle before
                disp('keeping obs avoid direction')                         % check, if there's still an obstacle in direction ant inteded to go
                obstacle_in_inteded_pos=find(cellfun(@(x) isequal(x,obj.dir_lookup{obj.intended_dir}), obj.fieldOfVision{4}), 1);
                
                if isempty(obstacle_in_inteded_pos)                         % if there's no obs in that direction
                    disp('keeping intended direction')
                    obj.obstacleEncountered=false;                          % change obj.obstacleEncountered flag
                    obj.direction=obj.intended_dir;                         % go in the intended direction
                    chosenNew_position=obj.dir_lookup{obj.direction};       % if dir that was intended when ant encountered obstacle is free, go this way
                end
                
            end
            
            % UPDATE POSITION
            % --------------------------------------------------------------------------------------------------------------------------
            
            obj.position=chosenNew_position+obj.position;   % update position of ant
            obj.nestDist=norm(obj.position-world.nest{1});  % update ant's distance to nest - THIS IS NOT AVAILABLE FOR PI, just for calculating normal random distr in random movements
            obj.trail{end+1}=obj.position;                  % update trail variable (append at the end)
            obj=pathIntegration(obj);                       % update global vector
        end
        
    end
end
