clc
clear variables
close all


%% Plots
reset(0)
set(0, 'DefaultFigureNumberTitle', 'off');
set(0, 'DefaultFigureWindowStyle', 'docked');
set(0, 'DefaultLineLineWidth', 1.2);
set(0, 'defaultaxesfontsize', 12);

figure('Name', 'Gate parking')
tiles = tiledlayout(1, 1);

simulation_tile = nexttile(tiles);
hold on
grid on
colororder({'k','r'})


idxChange = find(diff(out.simout(:, 1)) ~= 0);
firstIdx = [1; idxChange + 1];
lastIdx = [idxChange; numel(out.simout(:, 1))];

for region_idx = 1:numel(firstIdx)-1

    switch out.simout(firstIdx(region_idx), 1)
        case 1  % Red
            c = [1 0 0];
        case 2  % Yellow
            c = [1 1 0];
        case 3  % Green
            c = [0 1 0];
        otherwise
            c = [0.5 0.5 0.5];
    end

    if (region_idx == 4)
        firstIdx(region_idx) = firstIdx(region_idx) + 134;
    end

    xregion(out.tout(firstIdx(region_idx)), out.tout(lastIdx(region_idx)) + 0.2, 'FaceColor', c, 'HandleVisibility', 'off')

end


yyaxis left
plot(out.tout, out.simout(:, 3), '-k', 'DisplayName', 'Vehicle position');
ylabel('x [m]')

yyaxis right
plot(out.tout, out.simout(:, 2), '-r', 'DisplayName', 'Barrier angular position');
ylabel('alpha [deg]')

ylim padded

xlabel('Time [s]')

title('Parking gate simulation')
subtitle('Coordination of vehicle and gate based on semaphore')
legend()



%% Exports
exportgraphics(simulation_tile, '.latex/img/MATLAB/results.pdf');
