% Matthew Fugleberg

%% Open Quarter Car Test Model
mdl = 'testrig_quarter_car'; % Name of the Simulink car model
open_system(mdl)

sm_car_load_vehicle_data(mdl, '000'); % Load vehicle data (double wishbone setup)
sm_car_config_variants(mdl); % Configure the model

%% Open Suspension Model in Simulink
open_system('testrig_quarter_car/Linkage/DoubleWishbone', 'force')


%% Define Hardpoints to Optimize (x, y, z for each)
hardpoints = {
    'TrackRod', 'sInboard';
    'TrackRod', 'sOutboard';
    'LowerWishbone', 'sOutboard';
    'UpperWishbone', 'sOutboard';
    'LowerWishbone', 'sInboardF';
    'UpperWishbone', 'sInboardF';
    'LowerWishbone', 'sInboardR';
    'UpperWishbone', 'sInboardR';
    'Upright','sWheelCentre';
    
};

range_xyz = -0.01:0.01:0.01; % Small variation range

hp_list = [];
hp_index = 1;

for i = 1:size(hardpoints, 1)
    part_name = hardpoints{i, 1};
    point_name = hardpoints{i, 2};
    
    for k = 1:3  % X, Y, Z indices
        hp_list(hp_index).part = part_name;
        hp_list(hp_index).point = point_name;
        hp_list(hp_index).index = k;
        hp_list(hp_index).valueSet = ...
            Vehicle.Chassis.SuspA1.Linkage.(part_name).(point_name).Value(k) + range_xyz;
        hp_index = hp_index + 1;
    end
end

%% Define Metrics for Multi-Objective Optimization
metricNames = {'Bump Steer', 'Bump Camber'}; % Updated metric names
tgtValues = [0, -25.1]; % Target: Zero bump steer, 25.1° bump camber

%% Run Multi-Objective Optimization
[xFinal, fval, TSuspMetrics] = testrig_quarter_car_MOO(mdl, Vehicle, hp_list, metricNames, tgtValues);
