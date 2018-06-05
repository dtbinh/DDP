classdef leader_bot
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
        
        

        
    end %end of methods
end