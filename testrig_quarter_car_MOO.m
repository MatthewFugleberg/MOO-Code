function [xFinal, fval, TSuspMetricsFinal] = testrig_quarter_car_MOO(mdl, Vehicle, hp_list, metricNames, tgtValues)
% Multi-objective optimization for suspension kinematics using gamultiobj

open_system(mdl)
set_param(mdl, 'FastRestart', 'on')
out = sim(mdl);

disp('Metrics with Initial Hardpoints:')
TSuspMetricsStart = sm_car_testrig_quarter_car_plot1toecamber(out, true)

% Hold plot during optimization
fig_h = gcf;
hold(fig_h.Children(1), 'on');
hold(fig_h.Children(2), 'on');

%% Define Optimization Bounds and Initial Values
for hp_i = 1:length(hp_list)
    x0(hp_i) = Vehicle.Chassis.SuspA1.Linkage.(hp_list(hp_i).part).(hp_list(hp_i).point).Value(hp_list(hp_i).index);
    UB(hp_i) = hp_list(hp_i).valueSet(end);
    LB(hp_i) = hp_list(hp_i).valueSet(1);
end

%% Run Multi-Objective Optimization with gamultiobj
tOptStart = tic;

options = optimoptions('gamultiobj', ...
    'PopulationSize', 80, ...
    'MaxGenerations', 50, ...
    'CrossoverFraction', 0.8, ...
    'MutationFcn', @mutationadaptfeasible, ...
    'SelectionFcn', @selectiontournament, ...
    'Display', 'iter', ...
    'PlotFcn', {@gaplotpareto});

[x, fval] = gamultiobj(@(x)obj_MOO(x, mdl, hp_list, Vehicle, metricNames, tgtValues, fig_h), ...
    length(x0), [], [], [], [], LB, UB, options);

tOptDuration = toc(tOptStart);
disp(['Optimization completed in ' num2str(tOptDuration) ' seconds']);

%% Apply Optimized Values
xFinal = x(1, :);
for hp_i = 1:length(hp_list)
    Vehicle.Chassis.SuspA1.Linkage.(hp_list(hp_i).part).(hp_list(hp_i).point).Value(hp_list(hp_i).index) = xFinal(hp_i);
end
assignin('base', 'Vehicle', Vehicle);

% Evaluate final performance
out = sim(mdl);
disp('Final Metrics:')
[TSuspMetricsFinal, ~, ~] = sm_car_testrig_quarter_car_plot1toecamber(out, false)
set_param(mdl, 'FastRestart', 'off');

%% Plot Pareto Front
figure;
scatter(fval(:,1), fval(:,2), 'b', 'filled');
xlabel(metricNames{1}); ylabel(metricNames{2});
title('Pareto Front of Optimized Suspension');
grid on;
end
