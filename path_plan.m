function [ crunch ] = path_plan(map, botposition, target)
%We check if the map is on positive axis, otherwise we dont find the exact
%path as the gridmap doesnot reflect back to the orignal map coordinates
        if sum(sum(map))~=sum(sum(abs(map)))
            limsmin=abs(min(map));
            limsmin2=min(map);
            modmap=bsxfun(@plus,limsmin,map);
            target=target+limsmin;
            botposition= botposition+limsmin;
        else
            modmap=map;
            limsmin2=[0 0];
        end
    %% Bot sim intialization params
    mapper = BotSim(modmap);  %sets up a botSim object a map, and debug mode on.
    %% make grid
    limsMin = min(modmap); % minimum limits of the map
    limsMax = max(modmap); % maximum limits of the map
    
    dims = limsMax-limsMin; %dimension of the map
    res = 7; %sampling resouloution in cm
    iterators = dims/res;
    iterators = ceil(iterators)+[1 1]; %to counteract 1 based indexing
    mapArray = zeros(iterators); %preallocate for speed
    grid=[];
    %loops through the grid indexes and tests if they are inside the map
    kernel=[1 1 1 1 1 1 1 1]';
    hold on
    for i = 1:iterators(2)
        for j = 1:iterators(1)
            %get the testpositions
            testPos = limsMin + [j-1 i-1]*res;
            %create the neighbours matrix
            neighbours=[mapper.pointInsideMap(testPos) mapper.pointInsideMap(limsMin + [j-1 i]*res) mapper.pointInsideMap(limsMin + [j i-1]*res) mapper.pointInsideMap(limsMin + [j i]*res) mapper.pointInsideMap(limsMin + [j-2 i-1]*res) mapper.pointInsideMap(limsMin + [j-2 i]*res) mapper.pointInsideMap(limsMin + [j-1 i-2]*res) mapper.pointInsideMap(limsMin + [j-2 i-2]*res)];
            %Find the convolution, if the resultant is equal to two, then we
            %take the point else discard it

            if neighbours*kernel~=8
                mapArray(i,j)=1; 
            else
               mapArray(i,j)=0;
               elucedian_dist_target=sqrt((testPos(1)-target(1))^2+(testPos(2)-target(2))^2);
               elucedian_dist_start=sqrt((testPos(1)-botposition(1))^2+(testPos(2)-botposition(2))^2);
               point_dist=[testPos(1),testPos(2),elucedian_dist_start,elucedian_dist_target];
               grid=[grid;point_dist];

            end
        end
    end
    %% Because of the decimal shift we need to estimate the nearest 
    %element on the grid and use them as target and botposition 
    %for the path planning. So, Okay. First we need to find the 
    %closest point on grid to the bot position. To do this we 
    %first calulate the lowest four elements. This is done so that 
    %we can find the nearest point to bot position which also the 
    %closest to target. Get the minimum element of the thrid column 
    %of grid matrix which is the elucedian distance from the botPosition
    [~,I]=sort(grid(:,3)); %sort
    closest_four=grid(I([1:4]),:); %matrix to hold the closest four elements to bot_position
    %Next we find the closest point to bot_position which is also closest to target
    [M,I]=min(closest_four(:,4)); %closes element in column 4 which is the elucedian distance to target
    closest_point_to_bot_position=closest_four(I,:);


    %Simillarly we can find the closest point to target
    [M,I]=min(grid(:,4)); %closes element in column 4 which is the elucedian distance to target
    closest_point_to_target=grid(I,:);
    %Thus, we have estimated the closest points to target and bot_position on
    %grid. These are our A* search start and end points on the grid. Note that
    %these do not correstpond actual position of bot.
    %`closest_point_to_bot_position` is the start and `closest_point_to_target`
    % is the end.
    %Now we need to identify these points on the mapArray. This is quite straight forward
    botposition_maparray=abs([(closest_point_to_bot_position(:,[1,2])-limsMin)/res])+[1 1];
    target_maparray=abs([(closest_point_to_target(:,[1,2])-limsMin)/res])+[1 1];
    %% Path finding routine
    cream=find_path( mapArray, [botposition_maparray(2),botposition_maparray(1)], [target_maparray(2),target_maparray(1)]);

    %We need to convert maparray coordinates to actual grid coordinates
    crunch=[];
    for i=1:size(cream,1)
        crunch=[crunch; limsMin + (cream(i,:)-[1 1])*res];
    end
    
    %Finaly we add limsmin of the orignal map so that everything circles
    %back
 crunch=bsxfun(@plus,fliplr(limsmin2),crunch);
end