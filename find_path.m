function [ PathTake ] = find_path( grid, init, goal )
%% Search
cost=1;
Found=false;
Resign=false;

ActionTaken=zeros(size(grid)); %Matrix to store the action taken to reach that particular cell

OptimalPath(1:size(grid,1),1:size(grid,2))={' '}; %Optimal Path derived from A Star

%how to move in the grid
delta = [-1,  0; % go up
          0, -1; % go left
          1,  0; % go down
          0,  1; % go right
          1,  1; % diagonal down
          -1,-1; % diagonal up
          ];
%Populate a grid using the cell data structure define in the search class
for i=1:size(grid,1)
     for j=1:size(grid,2)
         gridCell=search(); %Create a cell
         if(grid(i,j)>0) %check if the grid point is explorable (1 means explorable)
             %Set the position (x,y).
             %Set heuristic value which is the elucedian distance from goal to
             %that grid cell.
             %Set the explorable property to 1.
             gridCell=gridCell.Set(i,j,1,sqrt((i-goal(1))^2+(j-goal(2))^2)); 
         else
             %Set the explorable property to 0.
             gridCell=gridCell.Set(i,j,0,sqrt((i-goal(1))^2+(j-goal(2))^2));
         end
         %Populate the object in the GRID matrix.
         GRID(i,j)=gridCell;
         clear gridCell;
     end
end
Start=search(); %Create a search object for botposition
%Set the x,y values of start
%Set the explorable status. This assumes that the bot can move around at
%the position. Thus the grid value can be directly assigned. 
%Set the heuristic. This is the elucedian distance from bot position to
%goal.
Start=Start.Set(init(1),init(2),grid(init(1),init(2)),sqrt((init(1)-goal(1))^2+(init(2)-goal(2))^2));
%We also know that this position is already visited so,
Start.isChecked=1;
GRID(Start.currX,Start.currY).isChecked=1;

%We do the same for the goal position
Goal=search();
Goal=Goal.Set(goal(1),goal(2),grid(goal(1),goal(2)),0);

%We start the search with the openlist which contains all the cell which
%need to be explore.
OpenList=[Start];

%We need this to represent the F score. initially the g value is zero.
small=Start.gValue+Start.hValue;

while(Found==false || Resign==false)
    small=OpenList(1).gValue+OpenList(1).hValue+cost; %update the F score with the whatever is the first element in the openlist
    
    %Decide which cell to explore next.
    for i=1:size(OpenList,2) %for all the elements in the openlist
        fValue=OpenList(i).gValue+OpenList(i).hValue; %compute the F socre of this cell
        if(fValue<=small) %if this cell is better then the previous
            small=fValue; %update the F score this this value
            ExpandNode=OpenList(i); %Thus we should explore this node further
            OpenListIndex=i; %Now we can remove this node from the openlist as we are going to expore this node next.
        end
    end
    
    OpenList(OpenListIndex)=[]; %Remove the node to be explored from the openlist.

    %Let the exploration begin. Han solo style!
    %We move according to the delta stategy.
    for i=1:size(delta,1) %for all the ways in which we can move
        direction=delta(i,:); %Choose a direction
        %Check if we can move in this direction
        %1. Check if we can explore this cell. I.e. is there a wall?
        %2. Check if the cell that we want to reach is inside the grid.
        if(ExpandNode.currX+ direction(1)<1 || ExpandNode.currX+direction(1)>size(grid,1)|| ExpandNode.currY+ direction(2)<1 || ExpandNode.currY+direction(2)>size(grid,2))
            continue;
        else
            %Now that we can reach the cell...
            NewCell=GRID(ExpandNode.currX+direction(1),ExpandNode.currY+direction(2)); %Create that cell
            if(NewCell.isChecked~=1 && NewCell.isEmpty~=1) %Check if we have not explored and visited this cell yet
                %Update the g value to be the cost plus the g value of the
                %node we are exploring.
                GRID(NewCell.currX,NewCell.currY).gValue=GRID(ExpandNode.currX,ExpandNode.currY).gValue+cost;
                GRID(NewCell.currX,NewCell.currY).isChecked=1; %We have just visited this cell, so we update this property
                OpenList=[OpenList,GRID(NewCell.currX,NewCell.currY)]; %put this cell same in openlist as we still need to explore it.
                %we took this action based on the delta strategy. This is updated so that we have the best action possible
                ActionTaken(NewCell.currX,NewCell.currY)=i;
            end
            %We also check if this cell is the goal
            if(NewCell.currX==Goal.currX && NewCell.currY==Goal.currY && NewCell.isEmpty~=1)
                Found=true;
                Resign=true;
                GRID(NewCell.currX,NewCell.currY).isChecked=1;
                GRID(NewCell.currX,NewCell.currY);
                break;
            end
        end
    end
    if(isempty(OpenList) && Found==false) %if openlist is empty and we were not able to reach the goal, we give up. Open the darn door.
         Resign=true;
         disp('I give up');
         break;
    end
end
PathTake=[]; % For storing the values taken for the path.
if (Found==true) %further process only if there is a path
    X=goal(1);Y=goal(2); %The goal
    while(X~=init(1)|| Y~=init(2)) %We go from the goal to target.
        %What action did we take at that cell.
        x2=X-delta(ActionTaken(X,Y),1);
        y2=Y-delta(ActionTaken(X,Y),2);
        %Appen the pathtake
        PathTake=[PathTake;[X,Y]];
        X=x2;
        Y=y2;
    end
    PathTake=[PathTake;[init(1),init(2)]]; % add the start state to the end
    %We flip the path taken. I dont know why this happens but i got this with hit and trial
    xes=fliplr((PathTake(:,2))');
    yes=fliplr((PathTake(:,1))');
    %instantiate the trajectory.
    coords=[];
    %populate the coords
    for i=1:size(xes,2)
        coords=[coords;[xes(i),yes(i)]];
    end
    %plot(coords(:,1),coords(:,2));
else
    disp('Kabooom: no path found');
end
end

