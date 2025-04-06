function F = obj_MOO(x, mdl, hp_list, Vehicle, metricNames, tgtValues, h_toeCamber)
% obj_bump_steer Objective function for multi-objective optimization
%   Runs a test with the new set of parameter values and calculates multiple performance metrics.
%
% Inputs:
%      mdl         - Simulink model name
%      hp_list     - Structure defining hardpoints to tune
%      Vehicle     - Vehicle data structure used by the model
%      metricNames - Cell array of metric names to optimize
%      tgtValues   - Target values for each metric
%      h_toeCamber - Figure handle for visualization
%
% Output:
%      F          - Vector of errors for multiple objectives (to minimize)
%
% Copyright 2020-2024 The MathWorks, Inc.

load_system(mdl);

% Update hardpoint values with optimizer's current set
for hp_i = 1:length(x)
    part_name  = hp_list(hp_i).part;
    hp_name    = hp_list(hp_i).point;
    index_val  = hp_list(hp_i).index;
    
    Vehicle.Chassis.SuspA1.Linkage.(part_name).(hp_name).Value(index_val) = x(hp_i);
end

% Assign updated structure to base workspace
assignin('base', 'Vehicle', Vehicle);

%% Simulate model
simOut = sim(mdl, 'StopTime', '40.4');

% Extract performance metrics
[TSuspMetrics, toeCurve, camCurve] = sm_car_testrig_quarter_car_plot1toecamber(simOut, false);

% Compute objective values
F = zeros(1, length(metricNames));
for i = 1:length(metricNames)
    metric_i = find(strcmp(TSuspMetrics.Names, metricNames{i}), 1, 'first'); % Ensure a single match
    if isempty(metric_i)
        error("Metric '%s' not found in TSuspMetrics.Names", metricNames{i});
    end
    F(i) = abs(TSuspMetrics.Values(metric_i) - tgtValues(i)); % Minimize deviation from target
end

%% Update existing plot
figure(h_toeCamber)
subplot(1, 2, 1); hold on; grid on;
plot(toeCurve.qToe, toeCurve.pzTire, 'LineWidth', 1);
title('Toe Curve');
xlabel('Toe (deg)'); ylabel('Suspension Travel (m)');

subplot(1, 2, 2); hold on; grid on;
plot(camCurve.qCam, camCurve.pzTire, 'LineWidth', 1);
title('Camber Curve');
xlabel('Camber (deg)');
end
