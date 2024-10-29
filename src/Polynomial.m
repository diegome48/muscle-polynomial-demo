classdef Polynomial
    %POLYNOMIAL Build and fit generalized, multivariate polynomials
    properties
        MAX_ALLOWED_COEFFICIENTS = 1e4;
        Nq      % Number of independent variables in the polynomial
        a       % Coefficients of the polynomial
        A       % Set of coefficient combinations
    end
    
    methods
        function obj = Polynomial(Nq, P)
            obj.Nq = Nq;
            cardinality = getCompleteACardinality(Nq, P);
            if cardinality > obj.MAX_ALLOWED_COEFFICIENTS
                msg = "The cardinality of A, %d, exceeds the maximum allowed size";
                error(msg, cardinality);
            end
            obj.A = zeros(cardinality, Nq);
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
            % TODO implement
        end
        
        % Computes the polynomial terms (without coefficients) from a pose
        % representation q.
        function t = computePolynomialTerms(obj, q)
            % TODO implement
        end
    end
end

function cardinality = getCompleteACardinality(N, P)
    cardinality = 0;
    for p = 0:P
        cardinality = cardinality + computeA(N, p, true);
    end
end

