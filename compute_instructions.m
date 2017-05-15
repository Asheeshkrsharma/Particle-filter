function [ instructions ] = compute_instructions( moves, botpose, botposition )

moves=flipud(moves);
instructions=[];
for i=1:size(moves,1)
    point=moves(i,:);
    %Calculate the slope
    angle = atan2(point(2) - botposition(2),point(1) - botposition(1));
    %Calculate difference
    turn=angle-botpose;
    %Calculate distance
    dist=sqrt((point(2) - botposition(2))^2+(point(1) - botposition(1))^2);
    %Store them
    instructions=[instructions; turn dist];
    %update the new pose
    botpose=botpose+turn;
    %set botposition
    botposition=point;
end
end

