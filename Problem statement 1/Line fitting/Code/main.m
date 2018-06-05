% This program tries to form a line using chapter 1 assumptions
%There is llimited sensor range thus there can be no cardinal number
%assigning
clear
close all
clc
%% Initialisation
% Parameters deciding number of bots
num_leader=2;
num_follower=10;
num_total=num_leader+num_follower;
agent(num_total)=agent_bot;  % All bots are defined together. the first bots are leader bots. latter ones are followers
% parameters deciding maximum velocity
max_lin_speed=1;
max_ang_speed=15;

time_step=0.005; % time step (normalisation to be done)

% parameters to find stopping condition

pos_threshold=0.008;
ang_threshold=0.16;

min_rad=0.1;  % minimum radius used for spiral
rad_inc=0.1; % How much to increment on completion of 2 pi revolution

sensor_range=0.2; % Radius which defines the sensor range
%% Initialising all bots
magnitude_scale=20;

% The command quiver helps in showing the orientation of the bots
figure(1)
for i=num_leader+1:num_total  % for followers
    agent(i).present_pos=[rand() rand() rand()*2*pi];
    agent(i).isleader=boolean(0);
    agent(i).isspiral=boolean(0);
    agent(i).radius=min_rad;
    quiver(agent(i).present_pos.x,agent(i).present_pos.y,cos(agent(i).present_pos.theta)/magnitude_scale,sin(agent(i).present_pos.theta)/magnitude_scale,'*r')
    hold on
end

for i=1:num_leader  % for leaders
    agent(i).present_pos=[rand() rand() rand()*2*pi];
    agent(i).isleader=boolean(1);
    agent(i).isspiral=boolean(0);
    agent(i).radius=min_rad;
    quiver(agent(i).present_pos.x,agent(i).present_pos.y,cos(agent(i).present_pos.theta)/magnitude_scale,sin(agent(i).present_pos.theta)/magnitude_scale,'*b')
    hold on
end


hold off
%% Taking global coordinates for leader bots as input

for i=1:num_leader
    x=input(strcat('Enter desired x coordinate for leader bot( ',num2str(i),')->'));
    y=input(strcat('Enter desired y coordinate for leader bot( ',num2str(i),')->'));
    theta=input(strcat('Enter desired orientation for leader bot( ',num2str(i),'). Enter value in degrees->'));
    theta=pi*theta/180;
    agent(i).desired_pos=[x y theta];
end
temp=[];
display('Entering while loop');
iter=0;
while(1) % infinite loop until deemed fit to break
%% Finding neighbourhood matrix and assigning values to agent structure
neighbours=zeros(num_total); % We create a matrix which will store the distances of all the robots with each other. the first num_leaders are of leader bots

for i=1:num_total
    for j=i+1:num_total
        [dist, diff_theta]=agent(i).present_pos.get_dist(agent(j).present_pos);
        neighbours(i,j)=dist;
        neighbours(j,i)=dist;
    end
end % end of finding neighbourhood matrix


for i=1:num_total
    agent(i).neighbours=find(neighbours(i,:)~=0 & neighbours(i,:)<=sensor_range);
    agent(i).neigh_dist=neighbours(i,agent(i).neighbours);
end % end of assignig neighbours to agents

%% Finding next position
for i=1:num_total
    if i<=num_leader %signifies movement for leader bots
    [x1, y1, theta1]=get_coord(agent(i).present_pos);
    [x2, y2, theta2]=get_coord(agent(i).desired_pos);
    vel=[x2-x1 y2-y1]*(max_lin_speed/norm([x2-x1 y2-y1]));
    omega=(theta2-theta1)*(max_ang_speed/abs(theta2-theta1));
    agent(i).present_vel=[vel(1),vel(2),omega];
    updated_pos=([x1, y1 ,theta1] + [vel,omega]*time_step);
    agent(i).present_pos=updated_pos;    
    else  %signifies movement for follower bots
        neigh_index=agent(i).neighbours;
        neigh_dist=agent(i).neigh_dist;
        num_neighbour=length(neigh_index);
        if length(neigh_index)>=2  % what to do if it does find neighbours
            agent(i).isspiral=boolean(0);
            agent(i).spiral_theta=0;
            agent(i).radius=0;
            x=zeros(1,num_neighbour);
            y=zeros(1,num_neighbour);
            for p=1:num_neighbour
             x(p)=agent(agent(i).neighbours(p)).present_pos.x;
             y(p)=agent(agent(i).neighbours(p)).present_pos.y;
            end
            
            [x1,y1,theta1]=get_coord(agent(i).present_pos);
            line_fit=polyfit(x,y,1);  % First value is slope, second value is intercept
            slope_1=line_fit(1);
            intercept_1=line_fit(2);
            slope_2=-1/slope_1; % slope for perpendicular line
            intercept_2=(x1/slope_1)+y1;  % equation found after calculating by hand
            points= ([-slope_1,1;-slope_2,1])\[intercept_1;intercept_2];
            agent(i).desired_pos=[points' theta1];
            
            
            [x1, y1, theta1]=get_coord(agent(i).present_pos);
    [x2, y2, theta2]=get_coord(agent(i).desired_pos);
    vel=[x2-x1 y2-y1]*(max_lin_speed/norm([x2-x1 y2-y1]));
    omega=(theta2-theta1)*(max_ang_speed/abs(theta2-theta1));
    agent(i).present_vel=[vel(1),vel(2),omega];
    updated_pos=([x1, y1 ,theta1] + [vel,omega]*time_step);
    agent(i).present_pos=updated_pos;
            
            
            
        else  % what to do if it doesnt find neighbours (remember to add spiral variable into the robots)
            if agent(i).isspiral==boolean(0)
                agent(i).isspiral=boolean(1);
                agent(i).radius=min_rad;                
                agent(i).spiral_theta=0;
                agent(i).present_vel=[0,max_lin_speed,0];
            end % end of if statement which checks whether follower is in spiral mode
            [x1, y1, theta1]=get_coord(agent(i).present_pos);
            accl_mag=(max_lin_speed^2)/agent(i).radius; 
            x_accl=-(accl_mag)*cos(agent(i).spiral_theta);
            y_accl=-(accl_mag)*sin(agent(i).spiral_theta);
            alpha=0;            
            agent(i).present_accl=[x_accl,y_accl,alpha];
            [x_vel,y_vel,omega]=get_vel(agent(i).present_vel);
            new_vel=[x_vel,y_vel,omega]+[x_accl,y_accl,alpha]*time_step;
            agent(i).present_vel=new_vel;
           % new_vel=new_vel*max_lin_speed/norm([new_vel(1),new_vel(2)]);
            accl=(new_vel-[x_vel,y_vel,omega])/time_step;
            new_pos=[x1,y1,theta1]+[x_vel,y_vel,omega]*time_step+0.5*accl*time_step*time_step;
            spi_theta=acos(dot([x_vel,y_vel],[new_vel(1),new_vel(2)])/(norm([x_vel,y_vel])*norm([new_vel(1),new_vel(2)])));
            agent(i).spiral_theta=agent(i).spiral_theta+spi_theta;
            agent(i).radius=agent(i).radius+(rad_inc*spi_theta/(2*pi));
            agent(i).present_pos=new_pos;
            
        end % End of movement for follower bots
        
    end % end of if statement to check if leader or follower bot
end % end of for loop for each agent


%% Plotting at the end of iteration
figure(2)
for i=num_leader+1:num_total
   quiver(agent(i).present_pos.x,agent(i).present_pos.y,cos(agent(i).present_pos.theta)/magnitude_scale,sin(agent(i).present_pos.theta)/magnitude_scale,'*r')
    hold on
end

for i=1:num_leader
    quiver(agent(i).present_pos.x,agent(i).present_pos.y,cos(agent(i).present_pos.theta)/magnitude_scale,sin(agent(i).present_pos.theta)/magnitude_scale,'*b')
    hold on
end


hold off
title(['Iteration',int2str(iter)]);
%j=j+1;
axis([0,1,0,1])
getframe;
iter=iter+1;
end % end of while loop