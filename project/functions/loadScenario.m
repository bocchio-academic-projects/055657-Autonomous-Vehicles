function [map, initial_state, final_state, img] = loadScenario(scenario_name)
% LOADMAP Loads a predefined map and corresponding initial/final states.
%
%   [MAP, INITIAL_STATE, FINAL_STATE, IMG] = LOADMAP(SCENARIO_NAME) loads a binary map
%   and associated start/end positions based on the provided SCENARIO_NAME.
%
%   The function searches for SCENARIO_NAME in a set of predefined cases. It reads the
%   original image (for visualization) and a processed grayscale image used as a
%   binary map for planning or navigation tasks. The initial and final states
%   are also defined manually based on the map.
%
%   Inputs:
%       SCENARIO_NAME - String identifying the map (e.g., '01', '02', ..., 'test').
%
%   Outputs:
%       MAP           - A binary occupancy grid (1 = free, 0 = obstacle).
%       INITIAL_STATE - Starting [row, col] position (1-indexed).
%       FINAL_STATE   - Goal [row, col] position (1-indexed).
%       IMG           - The original image for visualization.

switch scenario_name
    
    case '01'
        initial_state = [1, 12];
        final_state = [28, 12];

    case '02'
        initial_state = [1, 8];
        final_state = [27, 28];

    case '03'
        initial_state = [1, 1];
        final_state = [1, 30];

    case '04'
        initial_state = [1, 8];
        final_state = [19, 19];

        % initial_state = [11, 8];
        % final_state = [18, 8];

    case 'test'
        initial_state = [1, 8];
        final_state = [27, 28];
       
    case 'circle'
        initial_state = flip([8, 27]);
        final_state = flip([22, 27]);
        map = drawCircleOnMap([22, 15], 7.5);
        img = NaN;
        return

    case '3D_10'
        initial_state = [1, 1, 1];
        final_state = [24, 25, 30];
        map = load('map10.mat').scenario;
        img = NaN;
        return
       
    case '3D_20'
        initial_state = [1, 1, 1];
        final_state = [16, 27, 28];
        map = load('map20.mat').scenario;
        img = NaN;
        return

   case '3D_30'
        initial_state = [1, 1, 1];
        final_state = [30, 30, 30];
        map = load('map30.mat').scenario;
        img = NaN;
        return


    otherwise
        error('Unknown input map')

end

initial_state = flip(initial_state);
final_state = flip(final_state);

img = imread(['maps/original/' scenario_name '.png']);
map = rgb2gray(imread(['maps/resized/' scenario_name '.png'])) >= 0.95 * 255;
map(initial_state(1), initial_state(2)) = 1;
map(final_state(1), final_state(2)) = 1;

end


function map = drawCircleOnMap(center, radius)
% DRAWCIRCLEONMAP Draws a circular obstacle on a map.
%
%   MAP = DRAWCIRCLEONMAP(CENTER, RADIUS) creates a binary map of size 30x30,
%   with a circular obstacle centered at the specified position (CENTER) and
%   with the given radius. The obstacle is represented as 0, and free space as 1.
%
%   Inputs:
%       CENTER - 1x2 vector specifying the center of the circle [row, col].
%       RADIUS - Scalar defining the radius of the circle.
%
%   Outputs:
%       MAP - A binary occupancy grid (1 = free, 0 = obstacle).

    map = ones(30, 30);    
    [X, Y] = meshgrid(1:30, 1:30);
    dist = sqrt((X - center(2)).^2 + (Y - center(1)).^2);
    mask = dist <= radius;
    map(mask) = 0;
end
