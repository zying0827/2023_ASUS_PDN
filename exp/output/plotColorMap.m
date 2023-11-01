fp = fopen('voltageColorMap.txt');
colormap (jet (21));

numLayer = fscanf(fp, '%d', [1 1]);
numX = fscanf(fp, '%d', [1 1]);
numY = fscanf(fp, '%d', [1 1]);
numVia = fscanf(fp, '%d', [1 1]);


V = zeros(numVia, 3);
% V(i, 1): x, V(i, 2): y, V(i, 3): r
for via= 1: numVia
    V(via, 1) = fscanf(fp, '%f', [1 1]);
    V(via, 2) = fscanf(fp, '%f', [1 1]);
    V(via, 3) = fscanf(fp, '%f', [1 1]);
end


tiledlayout(1, numLayer)

for lay= 1: numLayer
    nexttile
    M = zeros(numY, numX);
    for x= 1: numX
        for y= 1: numY
            i = numY-y+1;
            j = x;
            M(i, j) = fscanf(fp, '%f', [1 1]);
        end
    end
    im = imagesc(M); 
    set(im, 'alphadata', M>0)
    
    hold on;
    
    for v= 1: numVia
        r = V(v, 3);
        c = [V(v, 1) V(v, 2)];
        pos = [c-r 2*r 2*r];
        r = rectangle('Position',pos,'Curvature',[1 1], 'FaceColor', 'red', 'Edgecolor','none');
    end
%    axis off;
end

c = colorbar;