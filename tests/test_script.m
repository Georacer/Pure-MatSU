% TEST_SCRIPT General testing script
% Used as a testing script container, to verify functionality of required components
% Run as-is, or by section
%
% Inputs:
%    (none)
%
% Outputs:
%    results - Structure containing test results

% Created at 2018/05/15 by George Zogopoulos-Papaliakos

%% Forward-Euler solver

clearvars -except test_name results
close all
clc
test_name = 'forw_euler_aircraft';
tic;

sim_options = simulation_options();

sim_options.solver.t_f = 1;
sim_options.solver.dt = 0.001;
sim_options.solver.solver_type = 0;

simulation(sim_options);

duration = toc;
results.(test_name).duration = duration;

%% ODE45 solver

test_name = 'ode45_aircraft';
clearvars -except test_name results
close all
clc
tic;

sim_options = simulation_options();

sim_options.solver.t_f = 30;
sim_options.solver.dt = 0.001;
sim_options.solver.solver_type = 1;

simulation(sim_options);

duration = toc;
results.(test_name).duration = duration;

%% ODE415s solver

clearvars -except test_name results
close all
clc
test_name = 'ode15s_aircraft';
tic;

sim_options = simulation_options();

sim_options.solver.t_f = 30;
sim_options.solver.dt = 0.001;
sim_options.solver.solver_type = 2;

simulation(sim_options);

duration = toc;
results.(test_name).duration = duration;

%% Results

clearvars -except test_name results
close all
clc

results
