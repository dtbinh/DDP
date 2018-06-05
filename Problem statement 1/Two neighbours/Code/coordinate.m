classdef coordinate
    properties
        x=0;
        y=0;
        theta=0;
    end    
    methods
        function [x1 y1 theta1]=get_coord(obj)
            x1=obj.x;
            y1=obj.y;
            theta1=obj.theta;
        end %get_coord
        function [distance diff_theta]=get_dist(obj,point)
            [x1, y1, theta1]=get_coord(point);
            diff_theta=theta1-obj.theta;
            distance=norm([x1 y1]-[obj.x obj.y]);            
        end %get_dist
        
    end %methods
end %class

