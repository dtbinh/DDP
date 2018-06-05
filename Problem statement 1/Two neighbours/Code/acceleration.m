classdef acceleration
    %This is the acceleration class
    %   Gives overall acceleration
    
    properties
        x_accl=0;
        y_accl=0;
        alpha=0;
    end % end of properties
    
    methods
        function [x1 y1 alpha1]=get_accl(obj)
            x1=obj.x_accl;
            y1=obj.y_accl;
            alpha1=obj.alpha;
        end %get_accl
    end% end of methods
    
end %end of class definition

