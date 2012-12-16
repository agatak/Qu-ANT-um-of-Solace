clear all

cd('C:\Users\Agata\Desktop\Mrowki')

% define length of simulation
end_time=1005;

% reply=input('Do you want to generate or load the map? 1 - generate map, 2
% - load map: ', 's'); if reply=='1'

% create universe
size=150;
universename='universe';
universe=World(size);
loc_obs=[];%{[8 30] [9 30] [10 30] [11 30] [12 30] [13 30] [14 30] [15 30] [16 30] [17 30] [18 30] [19 30] [20 30] [20 30] [21 30] [22 30] [23 30] [24 30] [25 30] [26 30] [27 30] [28 30] [29 30] [30 30] [31 30] [32 30] [33 30] [34 30] [35 30] [36 30] [37 30] [38 30] [39 30] [40 30] [41 30] [42 30]};
loc_land=[];%{[30 30] [20 20]};
loc_food=[];%{[40 45] [10 10]};
food_amount=5;
obs_amount=0;
land_amount=5;
ant_sight_range=15;
rotationerror=0.99; % from 0 to 1 is multiplied by good rotation
min_nest_dist=1/3;

% populate universe
universe=placeObstacles(universe,obs_amount,loc_obs);              % place obstacles in the world
universe=placeLandmarks(universe,land_amount,loc_land);            % place landmarks in the world
universe=placeFood(universe,food_amount,min_nest_dist,loc_food);   % place food in the world

%     reply=input('Do you want to save generated map? Y/N [Y]: ', 's'); if
%     reply=='Y'
%         name=input('Give the name of the file:' , 's');
%        save (name, universename);
%     elseif reply~='Y' && reply~='N'
%         disp('Wrong input. Not saving.')
%     end

% elseif reply=='2'
%     filename=uigetfile; load(filename)
%
% end




%create ant
b=Ant(1,universe,ant_sight_range,universe.nest{1}, rotationerror);
%c=Ant(1,universe,ant_sight_range,universe.nest{1},1);
b=lightCompass(b);
%c=lightCompass(c);
plotWorld(b,universe)

clear_counter=1;
for n=1:end_time
    
    b=walk(b,universe);
    %c=walk(c,universe);
    clear_counter=clear_counter+1;
    if clear_counter==30
        close gcf
        clear_counter=1;
    end
    
    plotWorld(b,universe);
    hold on
    
    
    quiver(b.position(1),b.position(2),b.dir_lookup{b.direction}(1), b.dir_lookup{b.direction}(2),0,'color',[1,1,1])
    if ~isempty(b.meanVector)
        quiver(b.position(1),b.position(2),b.meanVector(1), b.meanVector(2),0,'color',[1,0,1])
    end
    if ~isempty(b.feeder_meanVector)
        quiver(b.position(1),b.position(2),b.feeder_meanVector(1), b.feeder_meanVector(2),0,'color',[0,0,1])
    end
    plotAnt(b)
    %plotAnt(c) plotTrail(b) plotTrail(c) drawCircle(b)
    hold off
    pause(0.01);
    
    
    %k = waitforbuttonpress;
end