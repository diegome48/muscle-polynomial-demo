clc
clear variables
addpath src
% Degree of the considered polynomials
P = 3;

%% Definition of system and polynomial models
% System
body = Leg();
% A surrogate model per representation
modelTags = body.representations;
models = struct();
for i = 1:length(modelTags)
    tag = modelTags(i);
    Nq = length(body.getRepresentation(tag, true));
    models.(tag) = Polynomial(Nq, P);
end

%% Sampling body states - muscle length pairs
% Number of samples is 5 times (arbitrary) the number of coefficients in 
% the biggest polynomial (the full cartesian)
NSamples = 5 * models.fullCartesian.Na;
% Defining structures to store data from samples
samples = struct();
for i = 1:length(modelTags)
   tag = modelTags(i);
   Nq = models.(tag).Nq;
   samples.(tag) = zeros(NSamples, Nq);
end
muscleLengths = zeros(NSamples, 1);


% Sampling NSamples times
for m = 1:NSamples
    body.sample();
    muscleLengths(m) = body.getMuscleLength();
    for i = 1:length(modelTags)
       tag = modelTags(i);
       samples.(tag)(m,:) = body.getRepresentation(tag, true);
    end
end

%% Fitting the polynomials
for i = 1:length(modelTags)
   tag = modelTags(i);
   models.(tag).fit(samples.(tag), muscleLengths);
end

%% Creating a trajectory to test the models
% Defining knee flexions to evaluate the model in
Nm = 100;
kneeFlexions = linspace(0, 0.95*pi, Nm);
muscleLengths = zeros(length(kneeFlexions), 1);

% Allocating muscle length predictions
predictions = struct();
for i = 1:length(modelTags)
   tag = modelTags(i);
   predictions.(tag) = zeros(length(kneeFlexions), 1);
end

%% Evaluating the predictions for the trajecory
% We set it into a random position
body.sample();
for i = 1:length(kneeFlexions)
    body.setKneeFlexion(kneeFlexions(i));
    muscleLengths(i) = body.getMuscleLength();
    for tag_i = 1:length(modelTags)
       tag = modelTags(tag_i);
       q = body.getRepresentation(tag, true);
       predictions.(tag)(i) = models.(tag).evaluate(q);
    end
end

%% Plotting the different results
flexInDeg = kneeFlexions * 180 / pi;
hold off;
plot(flexInDeg, muscleLengths, LineWidth=3.0, Color="k");
hold on;
for i = 1:length(modelTags)
   tag = modelTags(i);
   plot(flexInDeg, predictions.(tag), LineWidth=2.0);
end
legend(["Geometric model", modelTags]);
title("Fitting from a training set");
xlabel("Knee flexion (deg)");
ylabel("Muscle length (m)");



%% Checking for errors by fitting directly to values to be predicted
% Allocating space for the samples
for i = 1:length(modelTags)
   tag = modelTags(i);
   Nq = models.(tag).Nq;
   samples.(tag) = zeros(Nm, Nq);
end

% Sampling across different knee flexions
for m = 1:Nm
    body.setKneeFlexion(kneeFlexions(m));
    for i = 1:length(modelTags)
       tag = modelTags(i);
       samples.(tag)(m,:) = body.getRepresentation(tag, true);
    end
end

% Fitting to the trajectory samples
for i = 1:length(modelTags)
   tag = modelTags(i);
   models.(tag).fit(samples.(tag), muscleLengths);
end

% Computing new set of predictions
for t = 1:length(modelTags)
    tag = modelTags(t);
    predictions.(tag) = zeros(length(kneeFlexions), 1);
    for i = 1:length(kneeFlexions)
        body.setKneeFlexion(kneeFlexions(i));
        q = body.getRepresentation(tag, true);
        predictions.(tag)(i) = models.(tag).evaluate(q);
    end
end

% Plotting
figure()
plot(flexInDeg, muscleLengths, LineWidth=3.0, Color="k")
hold on
for i = 1:length(modelTags)
   tag = modelTags(i);
   sep = 1e-4 * i;
   plot(flexInDeg, predictions.(tag) + sep, LineWidth=2.0);
end
legend(["Geometric model", modelTags]);
title("Fitting the evaluated set");
xlabel("Knee flexion (deg)");
ylabel("Muscle length (m)");
