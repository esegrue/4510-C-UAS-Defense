load adversary_paths_bank.mat
load big_island_map.mat
figure(1)
clf
show(flipud(occMap))
hold on
%surf(elevationMap.terrain.X,elevationMap.terrain.Y,elevationMap.terrain.Z)

for i = 1:length(adversary_paths_bank(1,1,:))
    pth = adversary_paths_bank{i};
    zs = zeros(1,length(pth(:,1))) + 10;
    plot3(pth(:,1),pth(:,2),zs)
end