function [avZ,range] = averageZ(rect)
z1 = rect(:,3);
avZ = mean(z1,1);
range = max(rect(:,3)) - min(rect(:,3));