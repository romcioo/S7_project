function rotPoints = rotateE(rotMat,points,reverse)
if reverse
    rotMat = rotMat';
end
rotPoints = zeros(size(points));
for i = 1:size(points,1) % for each point
    rotPoints(i,:) = (rotMat*points(i,:)')'; % convert points
end