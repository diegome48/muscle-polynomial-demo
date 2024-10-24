classdef Polynomial
    %POLYNOMIAL Build and fit generalized, multivariate polynomials
    properties
        Nq      % Number of independent variables in the polynomial
        a       % Coefficients of the polynomial
        A       % Set of coefficient combinations
    end
    
    methods
        function obj = Polynomial(Nq)
            obj.Nq = Nq;
            % TODO build A and initialize a to empty/zero vector
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
