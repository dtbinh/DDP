classdef follower_bot
    % This is a definition for the indepent glabally conscious leader
    % robots
    properties
        present_pos=coordinate;
        desired_pos=coordinate;  %typically, GPS coordinate
        present_vel=velocity; 
        
        present_accl=acceleration;
        collision_accl=acceleration;       
        desired_accl=acceleration;
        
        max_vision;  %potrays limitations of range sennsor
        
        neighbours=[]; % Indices of neighbours when necessary
        neigh_dist=[]; %Distances from the neighbours in the same order
    end %end of properties
    
    methods
        function obj=set.present_pos(obj,pos)
            obj.present_pos.x=pos(1);
            obj.present_pos.y=pos(2);
            obj.present_pos.theta=pos(3);
        end %Function to assign value to present position
        
        function obj=set.present_vel(obj,vel)
            obj.present_vel.x_vel=vel(1);
            obj.present_vel.y_vel=vel(2);
            obj.present_vel.omega=vel(3);
        end %function to assign value to present velocity
        
        function obj=set.desired_pos(obj,pos)
            obj.desired_pos.x=pos(1);
            obj.desired_pos.y=pos(2);
            obj.desired_pos.theta=pos(3);
        end %Function to assign value to desired position
        
        function obj=set.desired_accl(obj,accl)
            obj.desired_accl.x_accl=accl(1);
            obj.desired_accl.y_accl=accl(2);
            obj.desired_accl.alpha=accl(3);
        end %Function to assign value to desired accl
        
         function obj=set.present_accl(obj,accl)
            obj.present_accl.x_accl=accl(1);
            obj.present_accl.y_accl=accl(2);
            obj.present_accl.alpha=accl(3);
        end %function to assign value to present velocity
        
        
         function obj=set.collision_accl(obj,p)
             % p is the array with which we are concerned, max_threshold is
             % the threshold distance after which we dont care about the
             % collision. the constant is the proportionality constant
             constant=1;
           [dist ind]=min(obj.neigh_dist);
           if (dist<=obj.max_vision_distance)
               [x1 y1 z1]=obj.present_pos.get_coord();
               [x2 y2 z2]=p(ind).present_pos.get_coord();
               [vx1 vy1 vz1]=obj.present_vel.get_vel();
               [vx2 vy2 vz2]=p(ind).present_vel.get_vel();
               diff_pos=[x1-x2 y1-y2 z1-z2];
               diff_vel=[vx1-vx2 vy1-vy2 vz1-vz2];
               cos_theta=(diff_pos*diff_vel')/(norm(diff_pos*diff_vel'));
                if (cos_theta<0 || norm(diff_vel)==0)
                    obj.collision_accl.x_accl=0;
                    obj.collision_accl.y_accl=0;
                    obj.collision_accl.alpha=0;
                else
                    temp1=cross(diff_vel,diff_pos);
                    temp2=cross(diff_vel,temp1);
                    vel_direction=temp2/norm(temp2);
                    magnitude=constant*norm(diff_vel)/dist;
                    obj.collision_accl.x_accl=vel_direction(1)*magnitude;
                    obj.collision_accl.y_accl=vel_direction(2)*magnitude;
                    obj.collision_accl.alpha=vel_direction(3)*magnitude;
                    
                    
                end
               
           else
               obj.collision_accl.x_accl=0;
               obj.collision_accl.y_accl=0;
               obj.collision_accl.alpha=0;
           end         
           
         end % end of set.collision_accl
        
    end %end of methods
end