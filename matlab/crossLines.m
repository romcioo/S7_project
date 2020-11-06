function [xC,yC,valid] = crossLines(L1,L2,xLims)
mins = [min(xLims(1,:)) min(xLims(2,:))]';
maxs = [max(xLims(1,:)) max(xLims(2,:))]';
if isnan(L1(1)) && isnan(L2(1))
    xC = 0;
    yC = 0;
    if L1(2) == L2(2)
        valid = 1;
    else
        valid = 0;
    end
elseif isnan(L1(1))
    xC = L1(2);
    yC = xC*L2(1) + L2(2);
    if mins(2)<=xC && xC<=maxs(2)
        valid = 1;
    else
        valid = 0;
    end
elseif isnan(L2(1))
    xC = L2(2);
    yC = xC*L1(1) + L1(2);
    if mins(1)<=xC && xC<=maxs(1)
        valid = 1;
    else
        valid = 0;
    end
else
    xC = roots(L1 - L2);
    yC = L1(1)*xC + L1(2);
    valid = 0;
    if size(xC,2) == 0 || size(xC,1) == 0
        valid = 1;
    else
        if (mins(1)<=xC && xC<=maxs(1) && mins(2)<=xC && xC<=maxs(2))
            valid = 1;
        end
    end
end