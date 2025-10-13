classdef asset < handle
    properties
        location
    end
    methods
        function obj = asset(location)
            obj.location = [location(1), location(2)];
        end
    end
end