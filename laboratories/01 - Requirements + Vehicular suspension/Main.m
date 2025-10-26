clc
clear variables
close all

%% Parameters loading

run("parameters.m")

system = struct( ...
    'm', m, ...
    'r', r, ...
    'k', k);

force = struct( ...
    'road_excitation_omega', road_excitation_omega, ...
    'road_excitation_amplitude', road_excitation_amplitude);


%% ODE solution of the system

[t, z] = ode45(@(t, z) suspension_model(t, z, system, force), ...
    [0 10], ...
    [x0; v0], ...
    odeset('RelTol', 1e-6, 'AbsTol', 1e-6));


%% Plots

reset(0);
set(0, 'DefaultFigureNumberTitle', 'off');
set(0, 'DefaultFigureWindowStyle', 'docked');

figure_simulation_results = figure('Name', 'Simulation results');

nexttile;
hold on;
grid on;

plot(t, z(:,1))

xlabel('Time [s]');
ylabel('Vehicle vertical position [m]');
title('Vehicle suspension analysis over time');


%% Functions

function [z_dot] = suspension_model(t, z, system, force)

u(1) = force.road_excitation_amplitude * sin(force.road_excitation_omega * t);
u(2) = force.road_excitation_amplitude * force.road_excitation_omega * cos(force.road_excitation_omega * t);

z_dot(1, 1) = z(2);
z_dot(2, 1) = -system.r/system.m * z(2) + ...
    -system.k/system.m * z(1) + ...
    +system.r/system.m * u(2) + ...
    +system.k/system.m * u(1);

end
