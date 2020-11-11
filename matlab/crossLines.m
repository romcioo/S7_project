function [xC,yC,valid] = crossLines(L1,L2,xLims)
mins = [min(xLims(1,:)) min(xLims(2,:))]'; % get the minimums values of x
maxs = [max(xLims(1,:)) max(xLims(2,:))]'; % get the maximums values of x
if isnan(L1(1)) && isnan(L2(1)) % if both lines are vertical
    xC = 0;
    yC = 0;
    if L1(2) == L2(2) % if both cross the x at the same point
        valid = 1; % are the same line, so they cross
    else
        valid = 0;
    end
elseif isnan(L1(1)) % if only L1 is vertical
    xC = L1(2); % x cross
    yC = xC*L2(1) + L2(2); % y cross
    if mins(2)<=xC && xC<=maxs(2) % if x cross in limits of L2
        valid = 1; % they cross
    else
        valid = 0;
    end
elseif isnan(L2(1)) % if only L2 is vertical
    xC = L2(2);
    yC = xC*L1(1) + L1(2);
    if mins(1)<=xC && xC<=maxs(1) % if x cross in limits of L1
        valid = 1; % they cross
    else
        valid = 0;
    end
else % if neither of them are vertical
    xC = roots(L1 - L2); % get the x cross point
    yC = L1(1)*xC + L1(2); % get y cross
    valid = 0;
    if size(xC,2) == 0 || size(xC,1) == 0 % if they are both horitzontal
        valid = 1; % cross
    else
        if (mins(1)<=xC && xC<=maxs(1) && mins(2)<=xC && xC<=maxs(2)) % if the x is in limits of both lines
            valid = 1; % they cross
        end
    end
end