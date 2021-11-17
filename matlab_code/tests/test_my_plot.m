rows = 3;
cols = 1;

figure;
Rect    = [0.19, 0.07, 0.775, 0.845];
AxisPos = myPlotPos(cols, rows, Rect);
for i = 1:rows*cols
  hsub(i) = axes('Position', AxisPos(i, :));
end