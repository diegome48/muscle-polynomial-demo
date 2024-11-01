classdef Leg < handle
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
        %   - 3-vector of euler angles of the thigh in zyx formulation
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
        lb = [-1; -1; -1; 0; 0; 0; 0; 0];
        ub = [1; 1; 1; 2*pi; 2*pi; 2*pi; 3*pi/4; 3*pi/4];

        % Local coordinates joint locations / geometry
        thighKnee = [0; -0.25; 0];
        shankKnee = [0; 0.20; 0];
        shankAnkle = [0; -0.20; 0];
        footAnkle = [0; 0.10; 0];

        % Origin and insertion of "hamstring" in the thigh and shank
        % respectively, both in local coordinates
        origin = [0; 0.20; -0.05];
        insertion = [-0.05; 0.18; 0];

        % Available representations
        representations = ["fullCartesian", "absoluteRotation", ...
            "relativeRotation", "absoluteTranslation", ...
            "relativeTranslation", "inner", "kneeFlexion"];

    end
    
    methods
        function obj = Leg()
            obj.innerState = [0 1 0 0 0 0 pi/4 pi/2]';
        end
        
        function sample(obj)
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
            thighRotMat = eul2rotm(thighAngles');
            shankRotMat = eul2rotm([kneeFlex, 0, 0]) * thighRotMat;
            footRotMat = eul2rotm([-ankleFlex, 0, 0]) * shankRotMat;

            % Computing the translational coordinates of the shank and foot
            shankPos = thighPos + thighRotMat' * obj.thighKnee ...
                       - shankRotMat' * obj.shankKnee;
            footPos = shankPos + shankRotMat' * obj.shankAnkle ...
                       - footRotMat' * obj.footAnkle;
            
            % Assembling q
            q = [[thighPos; rotm2quat(thighRotMat)']...
                 [shankPos; rotm2quat(shankRotMat)']...
                 [footPos; rotm2quat(footRotMat)']];
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
            thighRotMat = eul2rotm(thighAngles');
            shankRotMat = eul2rotm([kneeFlex 0 0]) * thighRotMat;
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
            q = fullCartesian(4:7, :);
            if nargin == 2 && flatten
                q = q(:);
            end
        end

        function q = getRelativeRotationRepresentation(obj, flatten)
            absolute = obj.getAbsoluteRotationRepresentation();
            ref = absolute(:, 1);

            q = zeros(4, size(absolute, 2)-1);
            for col = 2:size(absolute, 2)
                p = absolute(:, col);
                q(:, col-1) = quaternionRelativeRotation(ref, p);
            end
            if nargin == 2 && flatten
                q = q(:);
            end
        end

        function q = getAbsoluteTranslationRepresentation(obj, flatten)
            fullCartesian = obj.getFullCartesianRepresentation();
            q = fullCartesian(1:3, :);
            if nargin == 2 && flatten
                q = q(:);
            end
        end

        function q = getRelativeTranslationRepresentation(obj, flatten)
            absoluteTrans = obj.getAbsoluteTranslationRepresentation();
            ref = absoluteTrans(:,1);
            colCount = size(absoluteTrans, 2)-1;
            q = zeros(3, colCount);
            for col = 1:colCount
                q(:, col) = absoluteTrans(:, col+1) - ref;
            end
            if nargin == 2 && flatten
                q = q(:);
            end
        end

        function j = getJointLocations(obj, flatten)
            % Computes the translational coordinates of the joint centers
            % Joint locations are computed twice, once with respect to each
            % involved segment.
            % Originally meant for debugging and Leg's plot method
            q = obj.getFullCartesianRepresentation(false);
            rThigh = q(1:3, 1);
            rShank = q(1:3, 2);
            rFoot = q(1:3, 3);
            thighRotMat = quat2rotm(q(4:7, 1)');
            shankRotMat = quat2rotm(q(4:7, 2)');
            footRotMat = quat2rotm(q(4:7, 3)');

            j = [rThigh + thighRotMat' * obj.thighKnee ...
                 rShank + shankRotMat' * obj.shankKnee ...
                 rShank + shankRotMat' * obj.shankAnkle ...
                 rFoot + footRotMat' * obj.footAnkle];
            if nargin == 2 && flatten
                j = j(:);
            end
        end

        function muscle = getOriginInsertion(obj)
            % Computes the translational coordinates of the origin and
            % insertion point of the muscle in the global reference frame.

            % Extracting state information
            q = obj.getFullCartesianRepresentation(false);
            rThigh = q(1:3, 1);
            rShank = q(1:3, 2);
            thighRotMat = quat2rotm(q(4:7, 1)');
            shankRotMat = quat2rotm(q(4:7, 2)');

            % Computing origin and insertion points
            originGlobal = rThigh + thighRotMat' * obj.origin;
            insertionGlobal = rShank + shankRotMat' * obj.insertion;

            muscle = [originGlobal insertionGlobal];
        end

        function rep = getRepresentation(obj, representation, flatten)
            % Gets any of the representations by its name
            switch representation
                case "fullCartesian"
                    rep = obj.getFullCartesianRepresentation(flatten);
                case "absoluteRotation"
                    rep = obj.getAbsoluteRotationRepresentation(flatten);
                case "relativeRotation"
                    rep = obj.getRelativeRotationRepresentation(flatten);
                case "absoluteTranslation"
                    rep = obj.getAbsoluteTranslationRepresentation(flatten);
                case "relativeTranslation"
                    rep = obj.getRelativeTranslationRepresentation(flatten);
                case "inner"
                    rep = obj.innerState;
                case "kneeFlexion"
                    rep = obj.innerState(7);
            end
        end

        function setKneeFlexion(obj, kneeFlexionAngle)
            obj.innerState(7) = kneeFlexionAngle;
        end

        function plot(obj)
            % Display the Leg's current state in 3d
            q = obj.getFullCartesianRepresentation(false);
            joints = obj.getJointLocations(false);
            chain = [2 * q(1:3, 1) - joints(:, 1)...
                     q(1:3, 1)...
                     joints(:, 1)...
                     joints(:, 2)...
                     q(1:3, 2)...
                     joints(:, 3)...
                     joints(:, 4)...
                     q(1:3, 3)...
                     2 * q(1:3, 3) - joints(:, 4)];
            muscles = obj.getOriginInsertion();
            figure();
            title("Leg pose")
            plot3(chain(1,:), -chain(3,:), chain(2,:), ...
                LineWidth=5.0, Color="black")
            hold on
            plot3(muscles(1,:), -muscles(3,:), muscles(2,:), ...
                LineWidth=4.0, Color="red")
            scatter3(joints(1,:), -joints(3,:), joints(2,:), 200, ...
                "filled")
            scatter3(q(1,:), -q(3,:), q(2,:), 200, "filled", ...
                MarkerFaceColor="black")
            axis equal
            xlabel("X")
            ylabel("Z")
            zlabel("Y")
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