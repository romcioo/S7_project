function [avZ,range] = averageZ(rect)
z1 = rect(:,3); % get all z
avZ = mean(z1,1); % average z
range = max(z1) - min(z1); % range of z