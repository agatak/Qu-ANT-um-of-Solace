classdef World
    properties 
        size                % world is composed of size x size fields
        nest                % location of the nest in the world
        food                % matrix containing location of the food sources in the world
        ants                % matrix containing location of all ants in the world and their velocity vectors
        landmarks           % matrix containing location of landmarks
        obstacles           % matrix containing location of fields that ants cannot pass through
        taken               % matrix with locations of all taken fields (free are 0, rest of numbers denote what type of object is in that spot)
    end
    
    methods (Access = public)
       %% Initialize world 
        function obj=World(n)
            obj.size = n;                                   % assign size to the world
            obj.nest{1}=[round(n/2),round(n/2)];            % place nest in the middle
            obj.taken=zeros(n,n);
            
            obj.taken(1,:)=ones(1,n)*4;                     % fill world boundaries with obstacles
            obj.taken(:,1)=ones(n,1)*4;                     % fill world boundaries with obstacles
            obj.taken(n,:)=ones(1,n)*4;                     % fill world boundaries with obstacles
            obj.taken(:,n)=ones(n,1)*4;                     % fill world boundaries with obstacles
            obj=fillTaken(obj,obj.nest,2);                  % call function that will mark spot of the nest as "taken" and unavailable for other items (landmarks, food, etc)
        end
        
        %% Fill taken
        function obj=fillTaken(obj,array,color)                   % function that marks locations stored in "array" as ones in obj.taken array (ones = taken and unavailable)
            size_array=numel(array);                        % check what size is the array
            for n=1:size_array
                obj.taken(array{n}(1),array{n}(2))=color;       % for each position stored in "array" mark corresponding point in obj.take as one
            end
        end
        
        
        %% Place landmarks
        function obj=placeLandmarks(obj,landmark_amount,loc_land)
             
            if isempty(loc_land)
                
                for n=1:landmark_amount
                    max_pos=obj.size;
                    istaken=1;
                    while istaken~=0                            % randomly generate location of a landmark and check if it is not taken. If it is, repeat
                        x_landmark=randi([1,max_pos]);
                        y_landmark=randi([1,max_pos]);
                        istaken=obj.taken(x_landmark,y_landmark);
                    end
                    obj.landmarks{1,n}=[x_landmark,y_landmark];
                end
               
            else
                for m=1:numel(loc_land)
                obj.landmarks{1,m}=loc_land{m};
                end
                landmark_amount=numel(loc_land);                % store the amount of landmarks
            end
            
            % assign a local vector to each landmark
            landmark_temp=[];
            for n=1:landmark_amount
                obj.landmarks{2,n}=obj.nest{1}-obj.landmarks{1,n};
                landmark_temp{n}=obj.landmarks{1,n};
            end
            obj=fillTaken(obj,landmark_temp,3);
        end        
        
        %% Place obstacles
        function obj=placeObstacles(obj,obs_amount,loc_obs)
            
            
            obj.obstacles=[];
              
            if isempty(loc_obs)
                
                for n=1:obs_amount
                    max_pos=obj.size;
                    istaken=1;
                    while istaken~=0                            % randomly generate location of a obstacle and check if it is not taken. If it is, repeat
                        x_obs=1+round((max_pos-2)*rand(1));
                        y_obs=1+round((max_pos-2)*rand(1));
                        istaken=obj.taken(x_obs,y_obs);
                    end
                    obj.obstacles{n}=[x_obs,y_obs];
                end
                   
                %now grow the obstacles into random direction

                obs_amount=numel(obj.obstacles);
                dir_lookup={[1,1],[0,1],[-1,1],[-1,0],[-1,-1],[0,-1],[1,-1],[1,0]};
                for m=1:obs_amount
                    current_obs=obj.obstacles{m};
                    growth_dir=randi([1,8]);                % direction 1 - right, 2- right up, 3 up an so on counter clockwise
                    growth_amount=randi([round(obj.size/8),round(obj.size/4)]);

                    for l=1:growth_amount

                        growth_field=[current_obs(1)+dir_lookup{growth_dir}(1),current_obs(2)+dir_lookup{growth_dir}(2)];
                        while growth_field(1)>obj.size || growth_field(2)>obj.size || growth_field(1)<=0 || growth_field(2)<=0
                            %disp('poza')
                            growth_dir=randi([1,8]);
                            growth_field=[current_obs(1)+dir_lookup{growth_dir}(1),current_obs(2)+dir_lookup{growth_dir}(2)];
                        end

                        istaken=obj.taken(growth_field(1),growth_field(2));

                        while istaken~=0 
                            %disp('zajete')
                            growth_dir=randi([1,8]);
                            growth_field=[current_obs(1)+dir_lookup{growth_dir}(1),current_obs(2)+dir_lookup{growth_dir}(2)];
                            istaken=obj.taken(growth_field(1),growth_field(2));
                        end

                        size_obs_matrix=numel(obj.obstacles);
                        obj.obstacles{size_obs_matrix+1}=growth_field;

                        % if it's growing at an angle (not vertical or
                        % horizontal) add another field, one to the right of
                        % the chosen one, to make it ant-tight
                        if norm(dir_lookup{growth_dir})>1
                            growth_field_bis=growth_field+[1 0];
                            size_obs_matrix=numel(obj.obstacles);
                            obj.obstacles{size_obs_matrix+1}=growth_field_bis;
                        end
  
                        current_obs=growth_field;    
                    end
                end
            
            else
                for m=1:numel(loc_obs)
                obj.obstacles{m}=loc_obs{m};
                end
            end
            
            obj=fillTaken(obj,obj.obstacles,4);
            
            %include edges as obstacles
            xx=(1:1:obj.size);
            yy=ones(1,obj.size);
            zz=yy*obj.size;
            top=[xx;yy];                            % coordinates of all points belonging to top boundary of the map
            bottom=[xx;zz];                         % coordinates of all points belonging to bottom boundary of the map
            left=[yy;xx];         % coordinates of all points belonging to left boundary of the map, without corners
            right=[zz;xx];        % coordinates of all points belonging to right boundary of the map, without corners
            for n=1:obj.size
                obj.obstacles{numel(obj.obstacles)+1}=[top(1,n),top(2,n)];
                obj.obstacles{numel(obj.obstacles)+1}=[bottom(1,n),bottom(2,n)];
                obj.obstacles{numel(obj.obstacles)+1}=[left(1,n),left(2,n)];
                obj.obstacles{numel(obj.obstacles)+1}=[right(1,n),right(2,n)];
            end
         end
        
         
        %% Place food
        function obj=placeFood(obj,food_amount,min_nest_dist,loc_food)
            
            if isempty(loc_food)
                max_nest_dist=floor(obj.size/2);    % define max distance so the food is not placed outside the world

                for n=1:food_amount                 % loop to randomly place food in the world but at a minimum distance from the nest

                    istaken=1;
                    while istaken~=0 || food_x_world<=0 || food_y_world<=0
                        r=randi([round(min_nest_dist*obj.size), max_nest_dist]);   % generate random r polar coordinate in between max/min distance
                        angle=degtorad(round(rand(1)*360));                 % gen random angle from 0 to 360 and convert to radians
                        [food_x,food_y]=pol2cart(angle,r);                  % convert polar coordinates to cartesian, with respect to nest location
                        food_x_world=round(food_x+round(obj.size/2));       % store food location, but with respect to [0,0] point of the world
                        food_y_world=round(food_y+round(obj.size/2));       % store food location, but with respect to [0,0] point of the world
                        if food_x_world==0
                            food_x_world=1;
                        end
                        if food_y_world==0
                            food_y_world=1;
                        end
                        istaken=obj.taken(food_x_world,food_y_world);       % check if the field is taken (have value of 1 in table obj.taken)
                    end

                    obj.food{n}=[food_x_world,food_y_world];
                    
                end
            else
                for m=1:numel(loc_food)
                obj.food{m}=loc_food{m};
                end
            end
            
            obj=fillTaken(obj,obj.food,5);
        end             
               
    end
end