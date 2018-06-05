% This program tries to form a line using chapter 0 assumptions and
% cardinal number assignment to the follower swarm bots.
% This program uses the function munkres() to solve the assignment problem
% using hungarian algorithm the copyrights of which are owned by Yi Cao and
% the notice for which has been reproduced wherever the funnction is used
% 
clear
close all
clc

% Parameters deciding number of bots
num_leader=2;
num_follower=15;
leader(num_leader)=leader_bot;
follower(num_follower)=follower_bot;
% parameters deciding maximum speed, accl and jerk
max_lin_speed=1.5;
max_lin_accl=20;

max_ang_speed=15;
max_ang_accl=20;

max_lin_jerk=5;
max_ang_jerk=5;

% parameter to find out velocity effect
linear_vel_constant=5;
angular_vel_constant=5;

time_step=0.01;

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
stop_threshold=0.1;

all_pos_norm=0;
all_vel_norm=0;
all_accl_norm=0;

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
%% Takinng global coordinates for leader bots as input
figure(2)
for i=1:num_leader
    x=input(strcat('Enter desired x coordinate for leader bot( ',num2str(i),')->'));
    y=input(strcat('Enter desired y coordinate for leader bot( ',num2str(i),')->'));
    theta=input(strcat('Enter desired orientation for leader bot( ',num2str(i),'). Enter value in degrees->'));
    theta=pi*theta/180;
    leader(i).desired_pos=[x y theta];
end
%% Here we start moving the robots
mode=7;
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
all_pos_norm=0;
all_vel_norm=0;
all_accl_norm=0;
%%% decides value of follower flag if mode is 1
if follower_start_flag==0
 if mode==1 && leader_stop_flag==1
    follower_start_flag=1; 
 end    
end
%%%%%%
%% decides trajectory of leader bot
if leader_stop_flag==0
    for i=1:num_leader
       %% getthing the desired acceleration for the leader_bot
        present=[leader(i).present_pos.x leader(i).present_pos.y leader(i).present_pos.theta];
        desired=[leader(i).desired_pos.x leader(i).desired_pos.y leader(i).desired_pos.theta];
        
        x1=present(1);
        y1=present(2);
        theta1=present(3);
        
        x2=desired(1);
        y2=desired(2);
        theta2=desired(3);
        
        linear_vel_direction=[x2-x1 y2-y1]/norm([x2-x1 y2-y1]);
        angular_vel_direction=(theta2-theta1)/abs(theta2-theta1);
        distance=norm([x2-x1 y2-y1]);
        
        linear_vel=min(linear_vel_constant*distance,max_lin_speed)*linear_vel_direction;
        angular_vel=min(angular_vel_constant*abs(theta2-theta1),max_ang_speed)*angular_vel_direction;
        
        desired_vel=[linear_vel angular_vel];
        present_vel=[leader(i).present_vel.x_vel leader(i).present_vel.y_vel leader(i).present_vel.omega];
        
        leader(i).desired_accl=desired_vel-present_vel;
        
        
        
    end
end % End of trajectory for leader flag
%% Decides trajectory of follower bots
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
     [x(i) y(i) theta(i)]=get_coord(leader(i).present_pos);    % stores present position of leader robots into variables x,y and theta
    end
    for i=1:num_follower
        desired_x(i)=x(1)+(i*(diff(x)/(num_follower+1)));
        desired_y(i)=y(1)+(i*(diff(y)/(num_follower+1)));
        desired_theta(i)=theta(1)+(i*(diff(theta)/(num_follower+1)));
    end
    for i=1:num_follower
        for k=1:num_follower
            [x_follower y_follower theta_follower]=get_coord(follower(k).present_pos);
            distance_mat(i,k)=norm([desired_x(i) desired_y(i)]-[x_follower y_follower]);
        end
    end
    distance_mat=distance_mat';
  %% This section assignes the desired position for each follower bot
  [assignment cost]=munkres(distance_mat);%Copyright (c) 2009, Yi Cao
                                          %All rights reserved.
   for i=1:num_follower
       follower(i).desired_pos=[desired_x(assignment(i)) desired_y(assignment(i)) desired_theta(assignment(i))];
   end
%% getting desired accl for follower bot
    if follower_stop_flag==0
     for i=1:num_follower
        % reading present and desired positions
        present=[follower(i).present_pos.x follower(i).present_pos.y follower(i).present_pos.theta];
        desired=[follower(i).desired_pos.x follower(i).desired_pos.y follower(i).desired_pos.theta];
        
        x1=present(1);
        y1=present(2);
        theta1=present(3);
        
        x2=desired(1);
        y2=desired(2);
        theta2=desired(3);
        % calculating parameters for wanted velocity
        linear_vel_direction=[x2-x1 y2-y1]/norm([x2-x1 y2-y1]);
        angular_vel_direction=(theta2-theta1)/abs(theta2-theta1);
        distance=norm([x2-x1 y2-y1]);
        
        linear_vel=min(linear_vel_constant*distance,max_lin_speed)*linear_vel_direction;
        angular_vel=min(angular_vel_constant*abs(theta2-theta1),max_ang_speed)*angular_vel_direction;
        % deciding desired speed at present position
        desired_vel=[linear_vel angular_vel];
        present_vel=[follower(i).present_vel.x_vel follower(i).present_vel.y_vel follower(i).present_vel.omega];
        
        follower(i).desired_accl=desired_vel-present_vel; % follower bot's desired velocity decided
        
     end
    end
    
end % End of trajectory for follower flag
%% This section executes all the necessary calculation to update the
%% positions and the velocities
% Updating for Leader

for i=1:num_leader
    if leader_stop_flag==0
   %%%%% Get values from the leader bot
    [x y theta]=get_coord(leader(i).present_pos);
    [x_vel y_vel omega]=get_vel(leader(i).present_vel);
    [x_accl y_accl alpha]=get_accl(leader(i).present_accl);
    %%%%% Update postion 
    x=x+(x_vel*time_step)+(0.5*x_accl*time_step*time_step);
    y=y+(y_vel*time_step)+(0.5*y_accl*time_step*time_step);
    theta=theta+(omega*time_step)+(0.5*alpha*time_step*time_step); 
    leader(i).present_pos=[x y theta];  %position updated
    %%%%% Update veolcity
    x_vel=x_vel+(x_accl*time_step);
    y_vel=y_vel+(y_accl*time_step);
    omega=omega+(alpha*time_step);
    leader(i).present_vel=[x_vel y_vel omega];  % velocity updated
    %%% Update acceleration
    [x2_accl y2_accl alpha2]=get_accl(leader(i).desired_accl);
    lin_accl_dir=[0 0];
    ang_accl_dir=0;
    
    if norm([x2_accl-x_accl y2_accl-y_accl])~=0  %%% This takes care of divide by zero error
    lin_accl_dir=[x2_accl-x_accl y2_accl-y_accl]/norm([x2_accl-x_accl y2_accl-y_accl]);
    end
    if alpha2-alpha~=0   %%%% This takes care of divide by zero error
    ang_accl_dir=(alpha2-alpha)/abs(alpha2-alpha);
    end
    
    x_accl=x_accl+min(norm([x2_accl-x_accl y2_accl-y_accl]),max_lin_jerk)*lin_accl_dir(1);
    y_accl=y_accl+min(norm([x2_accl-x_accl y2_accl-y_accl]),max_lin_jerk)*lin_accl_dir(2);
    alpha=alpha+min(abs(alpha2-alpha),max_ang_jerk)*ang_accl_dir;
    if norm([x_accl y_accl])>max_lin_accl  %%% makes sure that max acceleration is not breached
        temp_x=x_accl*max_lin_accl/norm([x_accl y_accl]);
        temp_y=y_accl*max_lin_accl/norm([x_accl y_accl]);
        x_accl=temp_x;
        y_accl=temp_y;
    end
    if abs(alpha)>max_ang_accl
        alpha=alpha*max_ang_accl/abs(alpha);
    end
    
    leader(i).present_accl=[x_accl y_accl alpha]; % updated accl
    end  
  [x_leader(i) y_leader(i) theta_leader(i)]=get_coord(leader(i).present_pos); %updating values to arrays
  [x_des y_des theta_des]=get_coord(leader(i).desired_pos); %getting desired values
  quiver(x_leader(i),y_leader(i),(cos(theta_leader(i))./magnitude_scale),(sin(theta_leader(i))./magnitude_scale),'*r');  % command for plotting
  hold on
 
%% here we find out if the leader bots are at rest
all_pos_norm=all_pos_norm+norm([x_leader(i) y_leader(i)]-[x_des y_des])+abs(theta_leader(i)-theta_des);
all_vel_norm=all_vel_norm+norm([x_vel y_vel])+abs(omega);  
all_accl_norm=all_accl_norm+norm([x_accl y_accl])+abs(alpha);
  
end%this is the end of updation loop for leader robots
%% checking if leader bot updations should stop
leader_stop_variable=(pos_constant*all_pos_norm)+(vel_constant*all_vel_norm)+(accl_constant*all_accl_norm);
if leader_stop_variable<=stop_threshold && leader_stop_flag==0
    leader_stop_flag=1;
end
% if (sum(isnan(theta_leader)))
%     break
% end
all_pos_norm=0;
all_vel_norm=0;
all_accl_norm=0;
%% Updating for follower

for i=1:num_follower
 if follower_stop_flag==0;
       %%%%% Get values from the follower bot
    [x y theta]=get_coord(follower(i).present_pos);
    [x_vel y_vel omega]=get_vel(follower(i).present_vel);
    [x_accl y_accl alpha]=get_accl(follower(i).present_accl);
    %%%%% Update postion 
    
    x=x+(x_vel*time_step)+(0.5*x_accl*time_step*time_step);
    
    y=y+(y_vel*time_step)+(0.5*y_accl*time_step*time_step);
    theta=theta+(omega*time_step)+(0.5*alpha*time_step*time_step); 
    follower(i).present_pos=[x y theta];  %position updated
    %%%%% Update veolcity
    x_vel=x_vel+(x_accl*time_step);
    y_vel=y_vel+(y_accl*time_step);
    omega=omega+(alpha*time_step);
    follower(i).present_vel=[x_vel y_vel omega];  % velocity updated
    %%% Update acceleration
    [x2_accl y2_accl alpha2]=get_accl(follower(i).desired_accl);
    lin_accl_dir=[0 0];
    ang_accl_dir=0;
    
    if norm([x2_accl-x_accl y2_accl-y_accl])~=0  %%% This takes care of divide by zero error
    lin_accl_dir=[x2_accl-x_accl y2_accl-y_accl]/norm([x2_accl-x_accl y2_accl-y_accl]);
    end
    if alpha2-alpha~=0   %%%% This takes care of divide by zero error
    ang_accl_dir=(alpha2-alpha)/abs(alpha2-alpha);
    end
    
    x_accl=x_accl+min(norm([x2_accl-x_accl y2_accl-y_accl]),max_lin_jerk)*lin_accl_dir(1);
    y_accl=y_accl+min(norm([x2_accl-x_accl y2_accl-y_accl]),max_lin_jerk)*lin_accl_dir(2);
    alpha=alpha+min(abs(alpha2-alpha),max_ang_jerk)*ang_accl_dir;
    if norm([x_accl y_accl])>max_lin_accl  %%% makes sure that max acceleration is not breached
        temp_x=x_accl*max_lin_accl/norm([x_accl y_accl]);
        temp_y=y_accl*max_lin_accl/norm([x_accl y_accl]);
        x_accl=temp_x;
        y_accl=temp_y;
    end
    if abs(alpha)>max_ang_accl
        alpha=alpha*max_ang_accl/abs(alpha);
    end
    
    follower(i).present_accl=[x_accl y_accl alpha]; % updated accl
    [x_des y_des theta_des]=get_coord(follower(i).desired_pos); %getting desired values
 end  
[x_follower(i) y_follower(i) theta_follower(i)]=get_coord(follower(i).present_pos);
quiver(x_follower(i),y_follower(i),(cos(theta_follower(i))./magnitude_scale),(sin(theta_follower(i))./magnitude_scale),'*b')

%% here we find out if the follower bots are at rest
all_pos_norm=all_pos_norm+norm([x_follower(i) y_follower(i)]-[x_des y_des])+abs(theta_follower(i)-theta_des);
all_vel_norm=all_vel_norm+norm([x_vel y_vel])+abs(omega);  
all_accl_norm=all_accl_norm+norm([x_accl y_accl])+abs(alpha);

end %This is the end of updation loop for follower bot

t=t+time_step;
%% Checks if the follower bots should stop
follower_stop_variable=(pos_constant*all_pos_norm)+(vel_constant*all_vel_norm)+(accl_constant*all_accl_norm);
if follower_stop_variable<=stop_threshold && leader_stop_flag==1
    follower_stop_flag=1;
end

if follower_stop_flag==1 && leader_stop_flag==1
    all_stop_flag=1;
end

hold off
title(['Iteration',int2str(j)]);
j=j+1;
axis([0,1,0,1])
getframe;
end