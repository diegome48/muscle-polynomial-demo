classdef Leg
    %LEG Geometrical representation of a leg consisting of three segments
    % The three segments (thigh, shank and foot) are connected by two
    % revolute joints (knee, ankle) for simplicity.
    % An instance of Leg can "sample" the state space via sample(), which 
    % will give feasible, random values to the inner state representation.
    % The constructor initializes the Leg instance by taking a sample.
    % The getMuscleLength() method computes the length of a supposed
    % hamstring muscle, which is just the euclidean distance between origin
    % and insertion.
    % The get<representation name>Representation() functions return the
    % different representations of the system as defined in the working
    % document.
    
    properties
        % Inner state representation (minimal set of coordinates)
        % A 8-vector containing, in this order,
        %   - 3-vector of translational coordinates of the thigh
        %   - 3-vector of euler angles of the thigh in xyz formulation
        %   - scalar with knee flexion angle in radians
        %   - scalar with ankle dorsi-flexion angle in radians
        innerState
        % Lower / Upper bounds for innerState
        % 8-vector in the same order as innerState.
        lb = [-1, -1, -1, 0, 0, 0, 0, 0];
        ub = [1, 1, 1, 2*pi, 2*pi, 2*pi, 3*pi/4, 3*pi/4];
        % TODO define joint locations / geometry
        % TODO define origin and insertion points for the muscle
    end
    
    methods
        function obj = Leg()
            obj.sample();
        end
        
        function sample(obj)
            %SAMPLE sets the state of the LEG to a random feasible point
            % TODO implement
            % NOTE if the beta angle is approx +-pi/2 then re-roll beta to
            % avoid gimbal lock
        end

        function q = getFullCartesianRepresentation(obj)
            % TODO implement
        end

        function l = getMuscleLength(obj)
            % TODO implement
        end

        function q = getAbsoluteRotationRepresentation(obj)
            % TODO implement
        end

        function q = getTranslationRepresentation(obj)
            % TODO implement
        end
    end
end

