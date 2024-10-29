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
        %   - 3-vector of translational coordinates xyz of the thigh
        %   - 3-vector of euler angles of the thigh in xyz formulation
        %   - scalar with knee flexion angle in radians
        %   - scalar with ankle dorsi-flexion angle in radians
        % When the thigh is in its default orientation (all euler angles 0)
        % and its local xyz axis are parallel to the global reference frame
        % the axis have the following (roughly) anatomical meaning
        %   x <---> anterior (+) posterior (-)
        %   y <---> proximal/up (+) distal/down (-)
        %   z <---> lateral (+) medial (-)
        % Hence knee flexion happens in the thigh's z axis, and ankle
        % flexion happens in the shank's z axis (note that all z axes are
        % parallel given that all rotations are in the z axis).
        innerState
        % Lower / Upper bounds for innerState
        % 8-vector in the same order as innerState.
        lb = [-1, -1, -1, 0, 0, 0, 0, 0];
        ub = [1, 1, 1, 2*pi, 2*pi, 2*pi, 3*pi/4, 3*pi/4];
        % Local coordinates joint locations / geometry
        thighKnee = [0 -0.25 0];
        shankKnee = [0 0.20 0];
        shankAnkle = [0 -0.20 0];
        footAnkle = [0 0.10 0];
        % TODO define origin and insertion points for the muscle
    end
    
    methods
        function obj = Leg()
            obj.sample();
        end
        
        function obj = sample(obj)
            %SAMPLE sets the state of the LEG to a random feasible point
            s = (obj.ub - obj.lb) .* rand(length(obj.lb), 1) - obj.lb;
            % Avoiding gimbal lock
            while isItGimballing(s)
                s = (obj.ub - obj.lb) .* rand(length(obj.lb), 1) - obj.lb;
            end
            obj.innerState = s;
        end

        function q = getFullCartesianRepresentation(obj)
            % Extracting innerState into readable vars
            thighPos = obj.innerState(1:3);
            thighAngles = obj.innerState(4:6);
            kneeFlex = obj.innerState(7);
            ankleFlex = obj.innerState(8);

            % Computing rotation matrices for each body
            rotMatThigh = eul2rotm(thighAngles, "XYZ");
            rotMatShank = eul2rotm([0 0 kneeFlex], "XYZ") * rotMatThigh;
            rotMatFoot = eul2rotm([0 0 ankleFlex], "XYZ") * rotMatShank;

            % Computing the translational coordinates of the shank and foot
            shankPos = thighPos + rotMatThigh' * obj.thighKnee ...
                       - rotMatShank' * obj.shankKnee;
            footPos = shankPos + rotMatShank' * obj.shankAnkle ...
                       - rotMatFoot' * obj.footAnkle;
            
            % Assembling q
            q = [thighPos; rotm2quat(rotMatThigh);
                 shankPos; rotm2quat(rotMatShank);
                 footPos; rotm2quat(rotMatFoot)];
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

function locked = isItGimballing(sample)
    EPSILON = pi / 36;
    beta = sample(5);
    locked = abs(abs(beta) - pi) < EPSILON;
end