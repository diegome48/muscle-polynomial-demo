classdef Polynomial < handle
    %POLYNOMIAL Build and fit generalized, multivariate polynomials
    properties
        MAX_ALLOWED_COEFFICIENTS = 1e4;
        Nq      % Number of independent variables in the polynomial
        Na      % Number of coefficients
        a       % Coefficients of the polynomial
        A       % Set of coefficient combinations
    end
    
    methods
        function obj = Polynomial(Nq, P)
            obj.Nq = Nq;
            obj.Na = getCompleteACardinality(Nq, P);
            if obj.Na > obj.MAX_ALLOWED_COEFFICIENTS
                msg = "The cardinality of A, %d, exceeds the maximum allowed size";
                error(msg, obj.Na);
            end
            obj.a = zeros(obj.Na, 1);
            obj.A = zeros(obj.Na, Nq);
            row = 1;
            for p = 0:P
                blockSize = computeA(Nq, p, true);
                obj.A(row:row+blockSize-1, :) = computeA(Nq, p);
                row = row + blockSize;
            end
        end
        
        % Fits the polynomial to fit the variable measures Q and the
        % corresponding muscle lengths l
        % Q is a matrix where every row is a pose representation q
        % l is a column vector with the muscle lenghts corresponding to the
        % states in Q.
        function fit(obj, Q, l)
            Nm = size(Q, 1);
            T = zeros(Nm, obj.Na);
            for row = 1:Nm
                T(row, :) = obj.computePolynomialTerms(Q(row, :));
            end
            % Cannot solve by pseudoinverse. Even though T has more rows
            % than columns, it is rank deficient, so the problem is
            % actually underdeterminate
            % obj.a = T\l;
            obj.a = lsqminnorm(T, l);
        end
        
        % Computes the polynomial terms (without coefficients) from a pose
        % representation q.
        function t = computePolynomialTerms(obj, q)
            t = zeros(1, obj.Na);
            for i = 1:obj.Na
                alpha = obj.A(i, :);
                t(i) = evaluateTerm(q, alpha);
            end
        end

        % Evaluate a set states Q.
        % Q must be a matrix where each row represents a state
        function outputs = evaluate(obj, Q)
            % If a column vector is provided, we flip it
            if size(Q, 2) == 1
                Q = Q';
            end
            Nm = size(Q, 1);
            outputs = zeros(Nm, 1);
            for row = 1:Nm
                polyTerms = obj.computePolynomialTerms(Q(row, :));
                outputs(row) = polyTerms * obj.a;
            end
        end
    end
end

% Computes a polynomial term from a vector of coordinates q and a
% coefficient vector alpha
function polyterm = evaluateTerm(q, alpha)
    polyterm = 1;
    for i = 1:length(q)
        power = alpha(i);
        if ~power
            continue
        end
        polyterm = polyterm * q(i)^power; 
    end
end

function cardinality = getCompleteACardinality(N, P)
    cardinality = 0;
    for p = 0:P
        cardinality = cardinality + computeA(N, p, true);
    end
end

