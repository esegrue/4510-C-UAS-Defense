classdef effector
    %EFFECTOR Class for defense
    %   Detailed explanation goes here

    properties
        location
        range
    end

    methods
        function obj = effector(location, range)
            %EFFECTOR Construct an instance of this class
            %   Detailed explanation goes here
            obj.location = location;
            obj.range = range;
        end
    end
end