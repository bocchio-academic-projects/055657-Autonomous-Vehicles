clc
clear variables
close all

%% Map parameters

map_size = [30, 30, 30];
N_obstacle = 30;


%% Map generation
map = ones(map_size);

for i = 1:N_obstacle
    sizeX = randi([3, 10]);
    sizeY = randi([3, 10]);
    sizeZ = randi([3, 10]);

    x = randi([1, map_size(1) - sizeX + 1]);
    y = randi([1, map_size(2) - sizeY + 1]);
    z = randi([1, map_size(3) - sizeZ + 1]);

    map(x:x+sizeX-1, y:y+sizeY-1, z:z+sizeZ-1) = 0;
end

% save(['map' num2str(N_obstacle) '.mat'], "scenario")


%% Plot

reset(0)
set(0, 'DefaultFigureNumberTitle', 'off');
set(0, 'DefaultFigureWindowStyle', 'docked');
set(0, 'DefaultLineLineWidth', 1.2);
set(0, 'defaultaxesfontsize', 12);

figure('Name', '3D Scenario')
tiles = tiledlayout(3, 6);

viewer_tile = nexttile(tiles, [3, 3]);
hold on
grid on

[x, y, z] = meshgrid(1:size(map, 2), 1:size(map, 1), 1:size(map, 3));
p = patch(isosurface(y, x, z, map, 0.5));
p.FaceColor = 'black';
p.EdgeColor = 'none';
camlight;
lighting gouraud;

daspect([1 1 1]);
view(3)

xlim([0 size(map, 1)+1])
ylim([0 size(map, 2)+1])
zlim([0 size(map, 3)+1])

title('3D Scenario with obstacles');
xlabel('X')
ylabel('Y')
zlabel('Z')


for z_idx = round(linspace(1, size(map, 3)-1, 9))

    nexttile(tiles)
    hold on

    imagesc(permute(map(:, :, z_idx), [2, 1]));

    axis equal tight;
    colormap(gray);

    title(['Z = ', num2str(z_idx)]);

    xlabel('X');
    ylabel('Y');

end