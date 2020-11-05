function [xC,yC,valid] = crossLines(L1,L2,xLims)
xC = roots(L1 - L2);
assignin('base','xC',xC);
yC = L1(1)*xC + L1(2);
valid = 0;
mins = [min(xLims(1,:)) min(xLims(2,:))]';
maxs = [max(xLims(1,:)) max(xLims(2,:))]';
if size(xC,2) == 0 || size(xC,1) == 0
    valid = 1;
else
    if (mins(1)<=xC && xC<=maxs(1) && mins(2)<=xC && xC<=maxs(2))
        valid = 1;
    end
end