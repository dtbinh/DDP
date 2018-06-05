% This program tries to form a line using chapter 1 assumptions
%There is llimited sensor range thus there can be no cardinal number
%assigning

% code status: End points are being correctly classified

% State of code: We have left appplication to angles, spiralling lines and
% collision
clear all
close all
clc
%% Initialisation
% Parameters deciding number of bots
num_leader=2;
num_follower=15;
num_total=num_leader+num_follower;
agent(num_total)=agent_bot;  % All bots are defined together. the first bots are leader bots. latter ones are followers
% parameters deciding maximum velocity
max_lin_speed=1.5;
max_ang_speed=15;
min_lin_speed=0.5;

time_step=0.005; % time step (normalisation to be done)

% parameters to find stopping condition

pos_threshold=0.008;
ang_threshold=0.16;

min_rad=0.1;  % minimum radius used for spiral
rad_inc=0.1; % How much to increment on completion of 2 pi revolution

sensor_range=0.2; % Radius which defines the sensor range
%%%% Constants for potential calculation
K_attractive_point=100;
K_attractive_point_spiral=5*K_attractive_point;
K_attractive_line=100;
settlement_factor_limit=0.01;
K_repulsive_point=2000;

collision_max_dist=sensor_range/3;

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
    agent(i).neigh_old=2*num_total;
end

for i=1:num_leader  % for leaders
    agent(i).present_pos=[rand() rand() rand()*2*pi];
    agent(i).isleader=boolean(1);
    agent(i).isspiral=boolean(0);
    agent(i).radius=min_rad;
    quiver(agent(i).present_pos.x,agent(i).present_pos.y,cos(agent(i).present_pos.theta)/magnitude_scale,sin(agent(i).present_pos.theta)/magnitude_scale,'*b')
    hold on
    agent(i).neigh_old=2*num_total;
end


hold off
%% Taking global coordinates for leader bots as input

lazy_test=input('Enter 1 for preprogrammed coordinates, 2 for manual entry->');
if lazy_test==2
for i=1:num_leader
    x=input(strcat('Enter desired x coordinate for leader bot( ',num2str(i),')->'));
    y=input(strcat('Enter desired y coordinate for leader bot( ',num2str(i),')->'));
    theta=input(strcat('Enter desired orientation for leader bot( ',num2str(i),'). Enter value in degrees->'));
    theta=pi*theta/180;
    agent(i).desired_pos=[x y theta];
end
else
    agent(1).desired_pos=[0.2,0.5,90];
    agent(2).desired_pos=[0.8,0.5,90];
end
temp=[];
display('Entering while loop');
iter=0;
%% Running the infinite loop
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




%% Finding Force acoording to vector field science

for i=1:num_total
    agent(i).force=[0,0];
     neigh_index=agent(i).neighbours;
     neigh_dist=agent(i).neigh_dist;
     num_neighbour=size(neigh_index);
     num_neighbour=num_neighbour(2);
    if i<=num_leader  % Movement for leader points
     %%%% Force due attraction from destination point
     %%%% Here Potential is calculated as V=K_attractive_point*(radius*transpose(radius)),
     %%%% where is the radius vector from the present_position to
     %%%% destinnation point. Thus when we take the derivative, we get
     %%%% force F=2*K_attractive potential*radius;
     [x1, y1, theta1]=get_coord(agent(i).present_pos);
     [x2, y2, theta2]=get_coord(agent(i).desired_pos);
     agent(i).force=agent(i).force+(2*K_attractive_point*([x2,y2]-[x1,y1]));
     %%% Force due repulsion from other robots and other obstacles
     
      for p=1:num_neighbour
          if neigh_dist(p)<=collision_max_dist
              [x2,y2,theta2]=get_coord(agent(neigh_index(p)).present_pos);
              agent(i).force=agent(i).force-(2*K_repulsive_point*([x2,y2]-[x1,y1])/(norm([x2,y2]-[x1,y1])^2));
          end
      end
     
    else  % Force calculation for follower bots
       % settlement_factor=2*settlement_factor_limit; % To ensure all bts have equal chance
        
        if num_neighbour>1 && agent(i).isendline==boolean(0)  % what to do if it does find enough neighbours
            agent(i).isspiral=boolean(0);
            agent(i).spiral_theta=0;
            agent(i).radius=0;
            % First we construct the line given by a linear fit of its
            % neighbours here we find the point where the normal from the
            % agent would drop to and assuming potential as
            % V=k*(perpendicular*transpose(perpendicular)) we get force as
            % F=2*k*perpendicular. Note: perpendicular is from robot to
            % line
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
            agent(i).force=agent(i).force+2*K_attractive_line*(points'-[x1,y1]);
            
            %%% This section is to add force as aresult of neighbour
            %%% repulsion
             for p=1:num_neighbour
          if neigh_dist(p)<=collision_max_dist
              [x2,y2,theta2]=get_coord(agent(neigh_index(p)).present_pos);
              agent(i).force=agent(i).force-(2*K_repulsive_point*([x2,y2]-[x1,y1])/(norm([x2,y2]-[x1,y1])^2));
          end
             end
            
            %%% This section will see if the agent is very close to line. Do note that this is done             
            settlement_factor=norm(points'-[x1,y1]);
            agent(i).settlement_factor=settlement_factor;
            if settlement_factor<=settlement_factor_limit  % If an agent seems to be near its line
                all_x=zeros(1,num_neighbour);
                all_y=zeros(1,num_neighbour);
                for p=1:num_neighbour
                    all_x(p)=agent(agent(i).neighbours(p)).present_pos.x-agent(i).present_pos.x;
                    all_y(p)=agent(agent(i).neighbours(p)).present_pos.y-agent(i).present_pos.y;
                end
                all_x=sort(all_x);
                all_y=sort(all_y);
                if all_x(1)*all_x(num_neighbour)>0 && all_y(1)*all_y(num_neighbour)>0  % Checks if the present agent bot is at the end of a line
                  agent(i).isendline=boolean(1);
                  agent(i).neigh_old=num_neighbour;
                else
                  agent(i).isendline=boolean(0);
                end
            end % End of checking settlement_factor
        else  % what to do if it doesnt find enough neighbours or the bot is in a line spiral
            if agent(i).isendline==boolean(1) 
            if num_neighbour<2 
                 agent(i).isendline=boolean(0);
                 agent(i).isspiral=boolean(0);
                 agent(i).force=[0,0];
             else
                all_x=zeros(1,num_neighbour);
                all_y=zeros(1,num_neighbour);
                for p=1:num_neighbour
                    all_x(p)=agent(agent(i).neighbours(p)).present_pos.x-agent(i).present_pos.x;
                    all_y(p)=agent(agent(i).neighbours(p)).present_pos.y-agent(i).present_pos.y;
                end
                all_x=sort(all_x);
                all_y=sort(all_y);
                if (all_x(1)*all_x(num_neighbour)<0 && all_y(1)*all_y(num_neighbour)<0) %|| agent(i).neigh_old<length(neigh_index)  % Checks if the present agent bot is at the end of a line
                  agent(i).isendline=boolean(0);
                 agent(i).isspiral=boolean(0);
                 agent(i).force=[0,0];
                end
            end
            end
             if agent(i).isspiral==boolean(0) 
                agent(i).isspiral=boolean(1);
                agent(i).radius=min_rad;
                
                if num_neighbour<2
                agent(i).spiral_theta=0;
                else
                    [x1,y1,theta1]=get_coord(agent(i).present_pos);
                    [x2,y2,theta2]=get_coord(agent(neigh_index(1)).present_pos);                    
                    length=norm([x1,y1]-[x2,y2]);
                    sine=(y1-y2)/length;
                    cosine=(x1-x2)/length;
                    if (x1-x2)~=0  % Non singular case                    
                    tangent=(y1-y2)/(x1-x2);                    
                    % quadrant 1
                    if sine>=0 && cosine>0 && tangent>=0
                        theta=acos(abs(cosine));
                    end
                    % quadrant 2
                    if sine>0 && cosine<=0 && tangent<0
                        theta=acos(abs(cosine));
                        theta=pi-theta;
                    end
                    % quadrant 3
                    if sine<=0 && cosine<0 && tangent>=0
                        theta=acos(abs(cosine));
                        theta=pi+theta;
                    end
                    % quadrant 4
                    if sine<0 && cosine>=0 && tangent<0
                        theta=acos(abs(cosine));
                        theta=(2*pi)-theta;
                    end
                    
                    else  % Singular case
                        if sine>0
                            theta=pi/2;
                        else
                            theta=3*pi/2;
                        end
                    end
                    
                    
                    agent(i).spiral_theta=theta;
                end
                agent(i).present_vel=max_lin_speed*[-sin(agent(i).spiral_theta),cos(agent(i).spiral_theta),0];
                agent(i).settlement_factor=-1;
                
                
              end  % end of commands if number of neighbours is less
              if agent(i).isendline==boolean(1)
                    [x1, y1, theta1]=get_coord(agent(i).present_pos);
                    accl_mag=(max_lin_speed^2)/agent(i).radius; 
                    x_accl=-(accl_mag)*cos(agent(i).spiral_theta);
                    y_accl=-(accl_mag)*sin(agent(i).spiral_theta);
                    alpha=0;            
                    agent(i).present_accl=[x_accl,y_accl,alpha];
                    [x_vel,y_vel,omega]=get_vel(agent(i).present_vel);
                    new_vel=[x_vel,y_vel,omega]+[x_accl,y_accl,alpha]*time_step;
                    temp_omega=omega;
                    temp_x_vel=new_vel(1);
                    temp_y_vel=new_vel(2);
                    velocity=[temp_x_vel,temp_y_vel];
                    norm_vel=(velocity*min(max(norm(velocity),min_lin_speed),max_lin_speed))/norm(velocity);
                    new_vel=[norm_vel,temp_omega];
                    agent(i).present_vel=new_vel;
           
                    accl=(new_vel-[x_vel,y_vel,omega])/time_step;
                    new_pos=[x1,y1,theta1]+[x_vel,y_vel,omega]*time_step+0.5*accl*time_step*time_step;
            
                    spi_theta=acos(dot([x_vel,y_vel],[new_vel(1),new_vel(2)])/(norm([x_vel,y_vel])*norm([new_vel(1),new_vel(2)])));
                    agent(i).spiral_theta=agent(i).spiral_theta+spi_theta;
                    agent(i).radius=agent(i).radius+(rad_inc*spi_theta/(2*pi));
                    agent(i).desired_pos=new_pos;
             %%%% Force here is calculated by imparting it speed neccesary
             %%%% reach to the next spiral point
                   [x1, y1, theta1]=get_coord(agent(i).present_pos);
                   [x2, y2, theta2]=get_coord(agent(i).desired_pos);
                   agent(i).force=agent(i).force+(([x2,y2]-[x1,y1])/time_step);
            
              end
             
        end % End of movement for follower bots
        
    end
end


%% Finding velocity and thus position as a result of the afore-applied Force calculations
% Here for the relation between velocity and force is given as F=m*v
% m is supposed mass and F is force. This is done as we assume that
% acceleration is near infinite.
for i=1:num_total
    force=agent(i).force;    
    if agent(i).isspiral==boolean(0) || agent(i).isendline==boolean(1)
    m=1;
    [x1,y1,theta1]=get_coord(agent(i).present_pos);
    
    velocity=force/m;
    if norm(velocity)~=0
    velocity=(velocity*min(max(norm(velocity),min_lin_speed),max_lin_speed))/norm(velocity);  % Makes sure that velocity is within a range
    end
    final_pos=[x1,y1,theta1]+[velocity,1]*time_step;
    agent(i).present_pos=final_pos;
    
    else  % This is the code for spiral movement
            [x1, y1, theta1]=get_coord(agent(i).present_pos);
            accl_mag=(max_lin_speed^2)/agent(i).radius; 
            x_accl=-(accl_mag)*cos(agent(i).spiral_theta);
            y_accl=-(accl_mag)*sin(agent(i).spiral_theta);
            alpha=0;            
            agent(i).present_accl=[x_accl,y_accl,alpha];
            [x_vel,y_vel,omega]=get_vel(agent(i).present_vel);
            new_vel=[x_vel,y_vel,omega]+[x_accl,y_accl,alpha]*time_step;
            agent(i).present_vel=new_vel;
           
            accl=(new_vel-[x_vel,y_vel,omega])/time_step;
            new_pos=[x1,y1,theta1]+[x_vel,y_vel,omega]*time_step+0.5*accl*time_step*time_step;
            spi_theta=acos(dot([x_vel,y_vel],[new_vel(1),new_vel(2)])/(norm([x_vel,y_vel])*norm([new_vel(1),new_vel(2)])));
            agent(i).spiral_theta=agent(i).spiral_theta+spi_theta;
            agent(i).radius=agent(i).radius+(rad_inc*spi_theta/(2*pi));
            agent(i).present_pos=new_pos;
    
    end
    
end
%% Plotting at the end of iteration
figure(2)
for i=num_leader+1:num_total  % for follower bots
    %theta=agent(i).present_pos.theta;
    theta=pi/2;
   if agent(i).isendline==boolean(0)
   quiver(agent(i).present_pos.x,agent(i).present_pos.y,cos(theta)/magnitude_scale,sin(theta)/magnitude_scale,'*r')
   else
       quiver(agent(i).present_pos.x,agent(i).present_pos.y,cos(theta)/magnitude_scale,sin(theta)/magnitude_scale,'*g')
   end
   circle(agent(i).present_pos.x,agent(i).present_pos.y,sensor_range);
    hold on
end

for i=1:num_leader   % for leader bots
   % theta=agent(i).present_pos.theta;
    theta=pi/2;
    quiver(agent(i).present_pos.x,agent(i).present_pos.y,cos(theta)/magnitude_scale,sin(theta)/magnitude_scale,'*b')
   % circle(agent(i).present_pos.x,agent(i).present_pos.y,sensor_range);
    hold on
end


hold off
title(['Iteration',int2str(iter)]);
%axis([0,1,0,1])
%axis([-0.5,1.5,-0.5,1.5])
%axis([-2,2,-2,2])
getframe;
iter=iter+1;
end % end of while loop