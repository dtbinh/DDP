classdef velocity
    %This class gives the linear velocity and angular velocity of a 2d AGENTof an agent
    
    
    properties
        x_vel=0;
        y_vel=0;
        omega=0;
    end %end of properties
    
    methods
        function [x1 y1 omega1]=get_vel(obj)
            x1=obj.x_vel;
            y1=obj.y_vel;
            omega1=obj.omega;
        end %get_vel
    end % end of methods
    
    
    
end %end of class
