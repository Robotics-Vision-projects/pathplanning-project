% Script for doing statistical analysis over the data obtained from the
% simulation of the RRT-connect algorithm for path planning in C++, using
% the RobWork libaries.
% The script slices the data into the different sampled epsilons, and
% calculates the relationship of this parameter with the number of steps
% and the time it took to do the calculation.
% Finally, it calculates the QQ-plot of the first and last set, in order to
% check if the results follow a normal distribution.

% The size of each epsilon sample must be specified in the following
% constant, and must correspond with the used value in the c++ code.
SAMPLE_SIZE = 100;
% Read the given dataset.
script_dir = fileparts(mfilename('fullpath'));
dataset_path = fullfile(script_dir, 'Datasets', 'path-statistics.txt');
% Check that the file exists
assert(exist(dataset_path, 'file') == 2, 'File does not exist')
% The data contains a 130x5 double struct ('X')
data = load(dataset_path);

% Get the mean of each parameter (size and time) for every used epsilon.
[rows, cols] = size(data);
sets = rows / SAMPLE_SIZE;
% Initialize the epsilons vector, and the mean vectors for the steps and
% times of the simulations.
epsilons = zeros(sets, 1);
steps = zeros(sets, 1);
times = zeros(sets, 1);
for i = 1:sets
    epsilons(i) = data(i*SAMPLE_SIZE, 1);
    steps(i) = mean(data((i-1)*SAMPLE_SIZE+1 : i*SAMPLE_SIZE, 2));
    times(i) = mean(data((i-1)*SAMPLE_SIZE+1 : i*SAMPLE_SIZE, 3));
end

% Get the total length of the calculated paths, doing an element-wise
% multiplication.
legnths = epsilons .* steps;

% Plot the evolution of the features, related to the algorithm epsilon.
figure;
subplot(1, 2, 1);
p = plot(epsilons, steps);
p(1).LineWidth = 2;
xlabel('RRT-connect algorithm {\epsilon}', 'Fontsize', 12);
ylabel('Mean number of algorithm steps', 'Fontsize', 12);
title('Evolution of the algorithm steps', ...
      'Fontsize', 12);
subplot(1, 2, 2);
p = plot(epsilons, times);
p(1).LineWidth = 2;
xlabel('RRT-connect algorithm {\epsilon}', 'Fontsize', 12);
ylabel('Mean number of algorithm time(s)', 'Fontsize', 12);
title('Evolution of the algorithm time', ...
      'Fontsize', 12);
  
figure;
p = plot(epsilons, legnths);
p(1).LineWidth = 2;
xlabel('RRT-connect algorithm {\epsilon}', 'Fontsize', 12);
ylabel('Mean number of path length (m)', 'Fontsize', 12);
title('Evolution of the total calculated length', ...
      'Fontsize', 12)

% Do the QQ-plot of the first set of values against a normal distribution.
data_1 = data(1:SAMPLE_SIZE, 2);
% Get 1000 values of a gaussian distribution with the same mean and sigma
% than the set.
gauss_1 = normrnd(mean(data_1), std(data_1), [1000, 1]);
figure
subplot(1, 2, 1);
qqplot(data_1, gauss_1);
title('QQ-plot of the step size for the first set');

% Do the QQ-plot of the last set of values against a normal distribution.
data_n = data((sets-1)*SAMPLE_SIZE+1 : sets*SAMPLE_SIZE, 2);
gauss_n = normrnd(mean(data_n), std(data_n), [1000, 1]);
subplot(1, 2, 2);
qqplot(data_n, gauss_n);
title('QQ-plot of the step size for the last set');
