function rotPoints = rotateE(angles,points,reverse)
angles = -angles; % set negative angles to make it work
rotm = eul2rotm(angles,'XYZ'); % get the rotation matrix
if reverse % if its necessary to go back to the global
    rotm = rotm^-1; % reverse
end

rotPoints = zeros(size(points));
for i = 1:size(points,1) % for each point
    rotPoints(i,:) = (rotm*points(i,:)')'; % convert points
end