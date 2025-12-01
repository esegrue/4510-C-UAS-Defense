classdef sensors < handle
    %SENSOR Class for sensors
    %   Detailed explanation goes here

    properties
        location
        range
        model
        params
        peakGain
        boresight
        beamwidth
        xg
        yg
        P
    end

    methods
        function obj = sensors(location, range, model, params, peakGain, boresight, beamwidth)
            %SENSOR Construct an instance of this class
            %   Detailed explanation goes here
            obj.location = location;
            obj.range = range;
            obj.model = model;
            obj.params = params;
            obj.peakGain = peakGain;
            obj.boresight = boresight;
            obj.beamwidth = beamwidth;
        end

        function obj = createAttenuationMap(obj)
            
        end

        function obj = createSensorContours(obj, mapSize)
            [xg, yg] = meshgrid(0:1:mapSize.horiz, 0:1:mapSize.vert);
            dx = xg - obj.location(1);
            dy = yg - obj.location(2);
            d = sqrt(dx.^2 + dy.^2);
            theta = atan2d(yg - obj.location(2), xg - obj.location(1));
            
            switch lower(obj.model)
                case 'logistic'
                    d50 = obj.params.d50;
                    k = obj.params.k;
                    Pd = 1 ./ (1 + exp((d - d50)/k));
                otherwise
                    error("Unknown sensor model")
            end

            bw = obj.beamwidth;
            if bw >= 360
                gain = ones(size(d));
            else
                % angular separation
                da = angdiff_deg(theta, obj.boresight); % use helper below
                % normalize to half-width: 0 at boresight, 1 at half-angle
                % use Gaussian-like or cos^n shape. We'll use Gaussian:
                sigma_ang = bw/2 / 1.177; % approx convert half-power to sigma
                gain = exp(-0.5 * (da./sigma_ang).^2);
            end
            P = obj.peakGain .* Pd .* gain;
            obj.xg = xg;
            obj.yg = yg;
            obj.P = Pd;
        end

        function da = angdiff_deg(theta1, theta2)
        % ANGDIFF_DEG Calculates the shortest angular difference between two angles
        % in degrees, handling the circular boundary at +/- 180 degrees.
        %
        % Usage:
        %   da = angdiff_deg(theta1, theta2)
        %
        % Inputs:
        %   theta1: The first angle(s) in degrees (can be a scalar or an array).
        %   theta2: The second angle(s) in degrees (can be a scalar or an array).
        %
        % Output:
        %   da: The shortest angular difference in degrees. This result will always 
        %       be in the range [-180, 180].
        
            % Calculate the raw difference
            da = theta1 - theta2;
        
            % Normalize the difference to the range [-180, 180]
            
            % 1. Bring the angle into the range [-360, 360] 
            % (Using mod 360 keeps the sign correct for negative inputs)
            da = mod(da + 180, 360) - 180;
        
            % If you want a non-negative difference (0 to 180), you would use:
            % da = abs(da);
        end

        function P_value = P_at_location(obj, adversary_pos)
            xUAS = adversary_pos(1);
            yUAS = adversary_pos(2);

            P_value = interp2(obj.xg, obj.yg, obj.P, xUAS, yUAS, 'linear');
        end
    end
end