% --- 3D Terrain Viewer ---
clear; clc; close all;

% 1. Load the map data
load('big_island_map.mat'); % This loads the 'elevationMap' variable

% Extract the grid data
X = elevationMap.terrain.X;
Y = elevationMap.terrain.Y;
Z = elevationMap.terrain.Z;

% 2. Create the 3D Surface Map
figure('Name', '3D Terrain View', 'Position', [200, 100, 900, 700]);

% Plot the surface
surf(X, Y, Z);

% 3. Enhance the 3D Visuals
shading interp;          % Smooths the surface and removes harsh grid lines
colormap('terrain');     % Applies a realistic topographical color palette
camlight left;           % Adds a light source from the left
lighting gouraud;        % Applies smooth lighting/shadows across the terrain

% 4. Format the Axes
title('3D Terrain View of Big Island Map', 'FontSize', 14);
xlabel('X Coordinate (m)', 'FontWeight', 'bold');
ylabel('Y Coordinate (m)', 'FontWeight', 'bold');
zlabel('Elevation (m)', 'FontWeight', 'bold');

% Set viewing angle and axis scaling
view(-45, 45);           % Sets a nice angled isometric perspective
axis tight;              % Snaps the bounding box to the data limits
grid on;

% Add a colorbar for elevation reference
c = colorbar;
c.Label.String = 'Elevation (m)';