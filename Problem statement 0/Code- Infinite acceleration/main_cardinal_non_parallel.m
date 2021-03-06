% This program tries to form a line using chapter 0 assumptions and
% cardinal number assignment to the follower swarm bots.
% This program uses the function munkres() to solve the assignment problem
% using hungarian algorithm the copyrights of which are owned by Yi Cao and
% the notice for which has been reproduced wherever the function is used
% 
clear
close all
clc

% Parameters deciding number of bots
num_leader=2;
num_follower=15;
leader(num_leader)=leader_bot;
follower(num_follower)=follower_bot;
% parameters deciding maximum velocity
max_lin_speed=1;
max_ang_speed=15;

time_step=0.005; % time step (normalisation to be done)
% parameters to find stopping condition

pos_threshold=0.008;
ang_threshold=0.16;

% global variable to store values of followers
x_follower=zeros(1,num_follower);
y_follower=zeros(1,num_follower);
theta_follower=zeros(1,num_follower);

% global variable to store values of leaders
x_leader=zeros(1,num_leader);
y_leader=zeros(1,num_leader);
theta_leader=zeros(1,num_leader);
theta_global=zeros(1,2);

% Parameters to decide the stopping criteria Stopping criteria is decided
% as
% (vel_constant*sum(all_vel))+(accl_constant*sum(all_accl))+(position_const
% ant*sum(all_position))<=threshold
vel_constant=1/10;
accl_constant=1/10;
pos_constant=1;
stop_threshold=0.05;



%% Initialising all bots
magnitude_scale=20;
% Initialising leader
% The command quiver helps in showing the orientation of the bots
figure(1)
for i=1:num_leader
    leader(i).present_pos=[rand() rand() rand()*2*pi];
    quiver(leader(i).present_pos.x,leader(i).present_pos.y,cos(leader(i).present_pos.theta)/magnitude_scale,sin(leader(i).present_pos.theta)/magnitude_scale,'*r')
    hold on
end

% Initialisation for followers
for i=1:num_follower
    follower(i).present_pos=[rand() rand() rand()*2*pi];
    quiver(follower(i).present_pos.x,follower(i).present_pos.y,cos(follower(i).present_pos.theta)/magnitude_scale,sin(follower(i).present_pos.theta)/magnitude_scale,'*b')
    
end
hold off
%% Taking global coordinates for leader bots as input
figure(2)
for i=1:num_leader
    x=input(strcat('Enter desired x coordinate for leader bot( ',num2str(i),')->'));
    y=input(strcat('Enter desired y coordinate for leader bot( ',num2str(i),')->'));
    theta=input(strcat('Enter desired orientation for leader bot( ',num2str(i),'). Enter value in degrees->'));
    theta=pi*theta/180;
    leader(i).desired_pos=[x y theta];
end
%% Here we start moving the robots
mode=7; % initialising mode to enter into the while loop below
while(mode~=1 && mode~=2)
mode=input('Enter value of mode.1 would imply that follower bots start moving only after leader reaches position. 2 means that all bots start moving togeher');
end
% Flags which tell if the respective robots have stopped moving
all_stop_flag=0; % if 1 means steady state has been reached
leader_stop_flag=0;  % if 1, means that steady state for leader bots has been reached
follower_start_flag=0; % if 1, means that the follower bots can move
follower_stop_flag=0; % if 1, means that follower bots can stop

%%%% Follower bots can move if mode is 2
if mode==2
    follower_start_flag=1;
end
t=0;
j=0;
while all_stop_flag==0

%%% decides value of follower flag if mode is 1
if follower_start_flag==0
 if mode==1 && leader_stop_flag==1
    follower_start_flag=1; 
 end    
end
%%%%%%


if follower_start_flag==1
    %% This section finds the distance matrix
    distance_mat=zeros(num_follower);
    x=zeros(1,num_leader);
    y=zeros(1,num_leader);
    theta=zeros(1,num_leader);
    desired_x=zeros(1,num_follower);
    desired_y=zeros(1,num_follower);
    desired_theta=zeros(1,num_follower);
    for i=1:num_leader
     [x(i), y(i), theta(i)]=get_coord(leader(i).present_pos);    % stores present position of leader robots into variables x,y and theta
    end
    for i=1:num_follower
        desired_x(i)=x(1)+(i*(diff(x)/(num_follower+1)));
        desired_y(i)=y(1)+(i*(diff(y)/(num_follower+1)));
        desired_theta(i)=theta(1)+(i*(diff(theta)/(num_follower+1)));
    end
    for i=1:num_follower
        for k=1:num_follower
            [x_follower, y_follower, theta_follower]=get_coord(follower(k).present_pos);
            distance_mat(i,k)=norm([desired_x(i) desired_y(i)]-[x_follower y_follower]);
        end
    end
    distance_mat=distance_mat';
  %% This section assignes the desired position for each follower bot
  [assignment, cost]=munkres(distance_mat);%Copyright (c) 2009, Yi Cao
                                          %All rights reserved.
   for i=1:num_follower
       follower(i).desired_pos=[desired_x(assignment(i)) desired_y(assignment(i)) desired_theta(assignment(i))];
   end
    
end % End of trajectory for follower flag
%% This section executes all the necessary calculation to update the
%% positions and the velocities
% Updating for Leader

leader_pos_norm=zeros(1,num_leader);
leader_ang_norm=zeros(1,num_leader);
for i=1:num_leader
    if leader_stop_flag==0
   %%%%% Get values from the leader bot
    [x, y, theta]=get_coord(leader(i).present_pos);
    [x_des, y_des, theta_des]=get_coord(leader(i).desired_pos);
    [x_vel, y_vel, omega]=get_vel(leader(i).present_vel);
    %%%%% Update postion 
    x=x+(x_vel*time_step);
    y=y+(y_vel*time_step);
    theta=theta+(omega*time_step); 
    leader(i).present_pos=[x y theta];  %position updated
   
    %%%%% Update veolcity
    x_vel=x_des-x;
    y_vel=y_des-y;
    omega=theta_des-theta;
    
    leader_pos_norm(i)=norm([x_vel,y_vel]);
    leader_ang_norm(i)=abs(omega);
    
    speed=norm([x_vel y_vel]);
    x_vel=x_vel*max_lin_speed/speed;
    y_vel=y_vel*max_lin_speed/speed;
    omega=sign(omega)*max_ang_speed;
    
    leader(i).present_vel=[x_vel y_vel omega];  % velocity updated
    
    
    end  
  [x_leader(i), y_leader(i), theta_leader(i)]=get_coord(leader(i).present_pos); %updating values to arrays
  [x_des, y_des ,theta_des]=get_coord(leader(i).desired_pos); %getting desired values
  quiver(x_leader(i),y_leader(i),(cos(theta_leader(i))./magnitude_scale),(sin(theta_leader(i))./magnitude_scale),'*r');  % command for plotting
  hold on
 


  
end%this is the end of updation loop for leader robots
%% checking if leader bot updations should stop

 if max(leader_pos_norm)<=pos_threshold && leader_stop_flag==0 && max(leader_ang_norm)<=ang_threshold
     leader_stop_flag=1;
     if follower_start_flag==0
         follower_start_flag=1;
     end
 end

follower_pos_norm=zeros(1,num_follower);
follower_ang_norm=zeros(1,num_follower);
%% Updating for follower

for i=1:num_follower
 if follower_stop_flag==0 && follower_start_flag==1
    [x, y, theta]=get_coord(follower(i).present_pos);
    [x_des, y_des, theta_des]=get_coord(follower(i).desired_pos);
    [x_vel, y_vel, omega]=get_vel(follower(i).present_vel);
    %%%%% Update postion 
    x=x+(x_vel*time_step);
    y=y+(y_vel*time_step);
    theta=theta+(omega*time_step); 
    follower(i).present_pos=[x y theta];  %position updated
   
    %%%%% Update veolcity
    x_vel=x_des-x;
    y_vel=y_des-y;
    omega=theta_des-theta;
    
    follower_pos_norm(i)=norm([x_vel,y_vel]);
    follower_ang_norm(i)=abs(omega);
    
    speed=norm([x_vel y_vel]);
    x_vel=x_vel*max_lin_speed/speed;
    y_vel=y_vel*max_lin_speed/speed;
    omega=sign(omega)*max_ang_speed;
    follower(i).present_vel=[x_vel y_vel omega];  % velocity updated

 end  
[x_follower(i), y_follower(i), theta_follower(i)]=get_coord(follower(i).present_pos);
quiver(x_follower(i),y_follower(i),(cos(theta_follower(i))./magnitude_scale),(sin(theta_follower(i))./magnitude_scale),'*b')
 

end %This is the end of updation loop for follower bot

%% here we find out if the follower bots are at rest
if max(follower_pos_norm)<=pos_threshold && follower_stop_flag==0 && leader_stop_flag==1 && follower_start_flag==1 && max(follower_ang_norm)<=ang_threshold
     follower_stop_flag=1;
end

t=t+time_step;
%% Checks if the follower bots should stop


if follower_stop_flag==1 && leader_stop_flag==1
    all_stop_flag=1;
end

hold off
title(['Iteration',int2str(j)]);
j=j+1;
axis([0,1,0,1])
getframe;
end