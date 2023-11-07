
isVoltage = 0;

%%
if isVoltage == 1
    fp = fopen('voltageColorMap.txt');
else
    fp = fopen('currentColorMap.txt');
end

fig = figure(1);
h = axes(fig, 'visible', 'off');

colormap (jet (256));
numNet = fscanf(fp, '%d', [1 1]);
numLayer = fscanf(fp, '%d', [1 1]);
numX = fscanf(fp, '%d', [1 1]);
numY = fscanf(fp, '%d', [1 1]);

xmin = fscanf(fp, '%f', [1 1]);
xmax = fscanf(fp, '%f', [1 1]);

numVia = fscanf(fp, '%d', [1 1]);
%%

V = zeros(numVia, 3);
% V(i, 1): x, V(i, 2): y, V(i, 3): r
for via= 1: numVia
    V(via, 1) = fscanf(fp, '%f', [1 1]);
    V(via, 2) = fscanf(fp, '%f', [1 1]);
    V(via, 3) = fscanf(fp, '%f', [1 1]);
end
%%

%tc = tiledlayout(1, numLayer);
tc = tiledlayout(1, 4, 'TileSpacing', 'Compact');
if  isVoltage == 1
    title(tc, 'Voltage', 'FontSize', 14);
else
    title(tc, 'Current', 'FontSize', 14);
end
%%

for lay= 1: numLayer
    nexttile;
    MAX(1:numY, 1:numX) = -1;
    
    for net = 1: numNet
        M = zeros(numY, numX);
        
        numGrid = fscanf(fp, '%d', [1 1]);
        
        for grid = 1: numGrid
            X = fscanf(fp, '%d', [1 1]);
            Y = fscanf(fp, '%d', [1 1]);
            val = fscanf(fp, '%f', [1 1]);
            
            M(numY-Y, X+1) = val;
        end
        
        MAX = max(MAX, M);
    end
    set(gca, 'color', 'k');
    x = 1: numX;
    y = 1: numY;
    if isVoltage == 1
        colormap(flipud(autumn));
    else
        colormap(flipud(winter));
    end
    %colormap(jet(256));
    im = imagesc(x, y, MAX);
    set(im, 'alphadata', MAX > 0);
    set(gca, 'dataAspectRatio', [1 1 1]);
    
    caxis([xmin xmax]);
    %set(gca, 'YDir', 'normal')
    hold on;
    for v= 1: numVia
        r = V(v, 3);
        c = [V(v, 1) numY - V(v, 2)];
        pos = [c-r 2*r 2*r];
        r = rectangle('Position', pos, 'Curvature', [1 1], 'FaceColor', 'black', 'Edgecolor','none');
    end
    title(sprintf('Layer %d', lay));
    %rectangle('position', [0, 0, 75 40], 'edgecolor', [1 0 0])
    %axis off;
    set(gca, 'XTick', []);
    set(gca, 'YTick', []);
    disp(lay);
end
%%
c = colorbar;
c.Layout.Tile = 'east';

%c.Title.String = "Voltage (V)";
if isVoltage == 1
    c.Title.String = "Voltage (V)";
else
    c.Title.String = "Current (A)";
end