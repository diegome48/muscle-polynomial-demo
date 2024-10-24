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
        thighPosition     % 3 translational coordinates xyz
        thighOrientation  % 3 euler angles, radians, xyz formulation
        kneeFlexion       % Knee flexion angle in radians
        ankleFlexion      % Ankle (plantar) flexion angle in radians
        % TODO setup bounds for the inner representation
    end
    
    methods
        function obj = Leg()
            obj.sample();
        end
        
        function sample(obj)
            %SAMPLE sets the state of the LEG to a random feasible point
            % TODO implement
        end

        function q = getFullCartesianRepresentation(obj)
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

