%% System parameters

% m (mass)
M = 2000; % [kg]
m = M/4; % [kg] 1/4 mass
v = 50/3.6; % [m/s] vehicle's speed

% k (spring)
delta_compression_max = 0.1;
k = m*9.81/delta_compression_max;
w = sqrt(k/m);

% r (damper)
h = 0.5;
r = h*2*m*w;


%% Force parameter (road roughness)

lambda = 5; % [m]
road_excitation_omega = 2*pi/lambda*v; % roughness frequency
road_excitation_amplitude = 0.1; % [m]


%% Initial condition
x0 = 0; % [m]
v0 = 0; % [m/s]


%% Clearing unused parameters

clear M v w h lambda delta_compression_max