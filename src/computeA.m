function A = computeA(N, P, maxSize)
%COMPUTEA Computes the set of all N-tuples of nonzero integers of sum L

    % First we solve the equivalent bars-and-stars (BS) problem
    nBS = P + N - 1;
    kBS = N - 1;
    cardinality = nchoosek(nBS, kBS);
    if nargin == 3 && cardinality > maxSize
        msg = "The cardinality of A, %d, exceeds the maximum allowed size";
        error(msg, cardinality);
    end
    barPosCombinations = nchoosek(1:nBS, kBS);

    % We translate the bars and stars solution to our original formulation
    A = zeros(cardinality, N);
    for row = 1:cardinality
        barPos = barPosCombinations(row, :);
        A(row, :) = barPosToCoefficients(barPos, P);
    end
end

function coefficients = barPosToCoefficients(barPositions, P)
    N = length(barPositions) + 1;
    barStarSum = N + P - 1;
    coefficients = zeros(1, N);
    
    % First and last bin sizes need to be computed separately
    coefficients(1) = barPositions(1) - 1;
    coefficients(N) = barStarSum - barPositions(end);
    for i = 2:length(barPositions)
        coefficients(i) = barPositions(i) - barPositions(i-1) - 1;
    end
end
