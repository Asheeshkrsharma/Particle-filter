function [botSim] = ParticleFilter(botSim, modifiedMap,target)
%%Convergence Results for 50 iterations and all noise levels.
%            averageCompletionTime    averageDisFromTgt    averagePathLength    percentCollision
%            _____________________    _________________    _________________    ________________

%    Map1    2.3249                   2.7988                96.67               0               
%    Map2    2.0617                   1.3657               93.435               0               
%    Map3     3.213                    5.289               129.65               0               

%% Map and resampe config
%We first analyse the map for its complexity to adapt the resampling
%strategy accordingly
[particles_needed, sampling_strategy,convergence_threshold] = map_profile(modifiedMap);
num=particles_needed;
movement_cost=1;

%% next we create a botsim object
scans=30; %30 is a good enough amount of scans
botSim.setMap(modifiedMap); %Usual stuff
botSim.setScanConfig(botSim.generateScanConfig(scans)); %Usual stuff
%% targetbot a.k.a India, this is the goal. we will use this to move in uncertainity (during localisation procedure).
India=BotSim(modifiedMap); %VascoDaGama want to go to India
India.setScanConfig(India.generateScanConfig(scans)); %Usual stuff
India.setBotPos(target); %Usual Stuff
%% Lets initiate
%Particle Filter Localisation Function
maxNumOfIterations = 50; %It should converge in 50 iterations, otherwise, we give up
particles(num,1) = BotSim; %how to set up a vector of objects
percentage_respawn=round(num*0.05); %This is how much we respawn particles
%Poor Vascodagama doesnt know where it is.
VascoDaGama=BotSim(modifiedMap); %Usual stuff
VascoDaGama.setScanConfig(VascoDaGama.generateScanConfig(scans)); %Usual stuff
for i = 1:num %Populate the particles in the map
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(10); %spawn the particles in random locations
    particles(i).setScanConfig(generateScanConfig(particles(i), scans)); %Usual stuff
    particles(i).setMotionNoise(0.2); %give the particles some motion noise
    particles(i).setTurningNoise(pi/180); %Usual stuff
end
n = 0; %Number of iterations taken
while(n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations   
    [botScan]= botSim.ultraScan(); %get a scan from the real robot.
    %% Write code for updating your particles scans
    weight = zeros(num,1); %Preacllocate for speed
    for i=1:num %for all the particles
        %if the particles are outside the map, we respawn them back in the
        %map. But do not weight them after respawn, as its just the waste
        %of time.
        if particles(i).insideMap() ==0
            particles(i).randomPose(10);
            weight(i)=0;
        else
        Scaneth= particles(i).ultraScan(); %To hold the scans of the current particle
        w=0; %Should be better than finding the max at the end, so we check this on each iteration
        var=4*std(Scaneth); %Not the best way to find the variance but it is still better then a constant value.
        
        for j=1:scans %For all the scans
            %% Write code for scoring your particles
            Scaneth = circshift(Scaneth,-1); %iterative shift
            diff= sqrt(sum((Scaneth-botScan).^2)); %difference
            likelee = 0.00000001 + (1/sqrt(2*pi*var))*exp(-((diff)^2/(2*var))); %estimated likelihood
            if likelee > w %Try to find the hieghest weight possible for the particles
                w=likelee;
                posit=j; %we also get the scan number which we can use to orient the paticle
            end
        end
        weight(i) = w; %Popoulate the weight
        %Essentially, we founf the highest scan which had the best weight
        %and outmatically orient it to that direction.
        particles(i).setBotAng(mod(particles(i).getBotAng() + posit*2*pi/scans, 2*pi));
        end
    end
    %% Write code for resampling your particles
    %now need to normalise
    weights = weight;
    if botSim.debug()
        figure(2)
        hold off;
        plot(weights./sum(weights)); %plot the normalized weights. It comes handy for analysing the distribution.
        figure(1)
    end

    indx=resampling(weights,num,sampling_strategy); %Use the particular resampling strategy according to the map profile
    positions = zeros(num, 2);  %Preallocate for speed
    angles = zeros(num,1); %Same as above
    %Update the position of all the particles, (Since we respawned all the particles outside the map back in and assigned a zero weight
    % those particles will get the best position, so we are fair).
    for i = 1:num %for all the particles
            positions(i,:)=particles(indx(i)).getBotPos(); %save the position for estimation
            angles(i)=particles(indx(i)).getBotAng(); %save the orientation for estimation
            particles(i).setBotPos(positions(i,:)); %Resampled location
            particles(i).setBotAng(angles(i)); %Resampled orientation
    end
    skip=0; %Not required, it is used to have a better check if we had r
    
    if skip==0
    %Pose is estimated as the mean pose of particles belonging to the largest position cluster.
    botpose=mean(angles);
    %Set the mean estimate
    botposition=mean(positions); %We can do much better with kmeans but for the task this is the simplest
    VascoDaGama.setBotPos(botposition); %Usual stuff
    VascoDaGama.setBotAng(botpose); %Usual stuff
    if botSim.debug()
        figure(1)
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'r'); %draw robot with line length 30 and green
        for i =1:num
            particles(i).drawBot(3); %draw particle with line length 3 and default color
        end
        VascoDaGama.drawBot(30, 'b');
        drawnow;
    end 
    end
    %% Write code to check for convergence 
    if skip==0
    std_dev=std(positions);
    %Convegence_threshold is calculated during the map profiling. A rule of
    %thumb is that more the polygon (i.e.) is convex, lower should be the
    %convergence threshold.

    %We also check the robot and the best hypothesis have simmilar scans 
    vascoscan=VascoDaGama.ultraScan(); %Scan from hypothesis
    botScan=botSim.ultraScan(); %Scan from actual robot
    vascoscan=abs(mean(abs(vascoscan))-mean(abs(botScan))); %Mean difference b/w the two. It doesnt matter if they have different orientation
    weight_deviation=std(weights./sum(weights)); %This is used to check our confidence. All particle (if) have the same wiehgt, out confidence will be high.
    if round(sum(std_dev)/2)<=convergence_threshold %We penalize the movement, if the robot actually could have got the correct estimate
        %So if the robot tries not to move drastically in next iteration.
        movement_cost=movement_cost*0.7; %Movement cost is cost high.
    end
    %disp(['Number of particles:',num2str(num),', Sampling srategy: ',num2str(sample),' confidence, ',num2str(weight_deviation)]);
    %We also take the the number iterations in account because, it might be
    %case that we localised incorrectly.
    %scan difference thershold is 0.7 which quite generous.
    if  round(sum(std_dev)/2)<=convergence_threshold && n>3 && abs(vascoscan)<=0.7
            if botSim.debug()
                disp('Summary:');
                if sampling_strategy==1
                    strat='Simplified PR';
                else
                    strat='Systematic';
                end
                disp(['Number of particles:',num2str(num),', Sampling srategy: ',strat,', confidence: ',num2str(weight_deviation),', Number of iterations: ',num2str(n)]);
                disp(['Standard deviation: ',num2str(round(sum(std_dev)/2)),', scan difference ',num2str(abs(vascoscan))]);
            end
          break; %We exit the loop and enter the path planning phase.
    end
    end
    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)	
    if skip==0
        for i=1:percentage_respawn
            particles(randi(num)).randomPose(10);
        end 
    end
    %% Write code to decide how to move next
    % 1st we have the target scan:
    [distances]=India.ultraScan();
    % 2nd we have the actual bot's scan
    [distances2]=botSim.ultraScan(); %get a scan from the real robot.
    %find the difference.
    disto=distances-distances2;
    %find the actual scan number
    [~,I]=min(disto); %Take the scans which are most aligned.
    botscan_dist=distances2(I,:); %Get the distance we need to travel
    turn = (I-1)*2*pi/scans ; %Calculate how much we need to turn. I is the scan number.
    move =  botscan_dist*movement_cost*rand()*0.4; %Movement is penalised. The '0.4' bit is really experimental.
    botSim.turn(turn); %turn the real robot.  
    botSim.move(move); %move the real robot. These movements are recorded for marking 
    for i =1:num %for all the particles. 
        particles(i).turn(turn); %turn the particle in the same way as the real robot
        particles(i).move(move); %move the particle in the same way as the real robot
    end  
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    if botSim.debug()
        figure(1)
        hold off; %the drawMap() function will clear the drawing when hold is off
        
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); %draw robot with line length 30 and green
        hold on;
        plot(target(1),target(2),'p','MarkerSize',12,'MarkerFaceColor','r');
        for i =1:num
            particles(i).drawBot(3); %draw particle with line length 3 and default color
        end
        drawnow;
     end
    if n>=maxNumOfIterations %We didnt converge in the given time frame. So quit.
        disp('Max number of iterations reached')
    end
end
%% Path planning
%intial position
botposition=VascoDaGama.getBotPos(); %No particular reason, just want the current esitmate
botpose=VascoDaGama.getBotAng(); %Same as above

oreo=path_plan(modifiedMap, botposition, target); %This is the path biscuit (Done using A* search).
oreo=[fliplr(target);oreo;fliplr(botposition);]; %Append the actual target and bot around the biscuit. This is done because 
%when path planning we are in the grid world, where we use the closesr
%point on the grid realtive to the the target and the 'real' real bot
%position.
if botSim.debug()
    hold on;
    plot(oreo(:,2),oreo(:,1));%path
end
oreo=[oreo(:,2) oreo(:,1)];
%We donot actually use the smooth trajectory but we plot it anyhow.
oreo2=smoothify(oreo);
if botSim.debug()
    hold on;
    plot(oreo2(:,2),oreo2(:,1));%path
end

instructions=compute_instructions( oreo,botpose,botposition); %compute the twists and moves we need to do, to follow the trajectory.
for i=1:size(instructions,1)
    instruction=instructions(i,:);
    botSim.turn(instruction(1));
    botSim.move(instruction(2));
    if botSim.debug()
    botSim.drawBot(1,'b');
    hold on;
    %pause(0.1);
    end
end
%% Drawing
%only draw if you are in debug mode or it will be slow during marking
if botSim.debug()
    figure(1)
    botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
    botSim.drawBot(30,'g'); %draw robot with line length 30 and green
    VascoDaGama.drawBot(30,'b'); %draws the mean ghost bot with line length 30 and red
    target_bot=BotSim(modifiedMap);
    target_bot.setBotPos(target);
    [~,target_ultrascan]=target_bot.ultraScan();
    target_bot.drawBot(30,'y')
    drawnow;
end
end

