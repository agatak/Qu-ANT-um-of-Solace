function plotWorld(b,universe)
%plot
hold on
xlim([1 universe.size]) 
ylim([1 universe.size])
colormap ([0 0.5 0; 1 1 1; 1 0 0; 0 0 0; 1 1 0])
image(universe.taken')
axis ('square')
hold off
end