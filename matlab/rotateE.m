function rotPoints = rotateE(mat,points,reverse)
if reverse
    mat = mat';
end
rotPoints = zeros(size(points));
for i = 1:size(points,1) % for each point
    rotPoints(i,:) = (mat*points(i,:)')'; % convert points
end