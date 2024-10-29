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

        % Origin and insertion of "hamstring" in the thigh and shank
        % respectively, both in local coordinates
        origin = [0 0.20 -0.05];
        insertion = [-0.05 0.18 0]

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

        function q = getFullCartesianRepresentation(obj, flatten)
            % Computing the full cartesian representation of the state
            % given by obj.innerState.
            % For convenience the function returns a matrix where each
            % column is the representation of a single segment. To obtain a
            % vector just flatten the output by setting flatten = true
            % Extracting innerState into readable vars
            thighPos = obj.innerState(1:3);
            thighAngles = obj.innerState(4:6);
            kneeFlex = obj.innerState(7);
            ankleFlex = obj.innerState(8);

            % Computing rotation matrices for each body
            thighRotMat = eul2rotm(thighAngles, "XYZ");
            shankRotMat = eul2rotm([0 0 kneeFlex], "XYZ") * thighRotMat;
            footRotMat = eul2rotm([0 0 ankleFlex], "XYZ") * shankRotMat;

            % Computing the translational coordinates of the shank and foot
            shankPos = thighPos + thighRotMat' * obj.thighKnee ...
                       - shankRotMat' * obj.shankKnee;
            footPos = shankPos + shankRotMat' * obj.shankAnkle ...
                       - footRotMat' * obj.footAnkle;
            
            % Assembling q
            q = [[thighPos; rotm2quat(thighRotMat)]...
                 [shankPos; rotm2quat(shankRotMat)]...
                 [footPos; rotm2quat(footRotMat)]];
            if nargin == 2 && flatten
                q = q(:);
            end
        end

        function l = getMuscleLength(obj)
            % Extracting relevant coordinates
            thighPos = obj.innerState(1:3);
            thighAngles = obj.innerState(4:6);
            kneeFlex = obj.innerState(7);
            
            % Determining state of thigh and shank
            thighRotMat = eul2rotm(thighAngles, "XYZ");
            shankRotMat = eul2rotm([0 0 kneeFlex], "XYZ") * thighRotMat;
            shankPos = thighPos + thighRotMat' * obj.thighKnee ...
                       - shankRotMat' * obj.shankKnee;

            % Computing positions of origin, insertion in global ref.
            originPos = thighPos + thighRotMat' * obj.origin;
            insertionPos = shankPos + shankRotMat' * obj.insertion;

            l = norm(insertionPos - originPos);
        end

        function q = getAbsoluteRotationRepresentation(obj, flatten)
            % This should clearly be cached, too much ATM
            fullCartesian = obj.getFullCartesianRepresentation();
            q = fullCartesian(4:6, :);
            if nargin == 2 && flatten
                q = q(:);
            end
        end

        function q = getRelativeRotationRepresentation(obj)
            absolute = obj.getAbsoluteRotationRepresentation();
            ref = absolute(:, 1);

            q = zeros(4, size(absolute, 2)-1);
            for col = 2:size(absolute, 2)
                p = absolute(:, col);
                q(:, col-1) = quaternionRelativeRotation(ref, p);
            end
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

function pij = quaternionRelativeRotation(pi, pj)
    % As in Nikravesh 6.122
    ej0 = pj(1);
    ej = pj(2:4);
    Lj = [-ej, -skew(ej) + ej0 * eye(3)];
    LjStar = [pj'; Lj];
    pij = LjStar * pi;
end

function es = skew(e)
    % Skew matrix from a 3-vector
    es = [0, -e(3), e(2);
          e(3), 0, -e(1);
         -e(2), e(1), 0];
end