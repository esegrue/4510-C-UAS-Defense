classdef sensors
    %SENSOR Class for sensors
    %   Detailed explanation goes here

    properties
        location
        range
    end

    methods
        function obj = sensors(location, range)
            %SENSOR Construct an instance of this class
            %   Detailed explanation goes here
            obj.location = location;
            obj.range = range;
        end
    end
end