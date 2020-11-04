function rotPoints = rotateE(angles,points,reverse)
angles = -angles;
rotm = eul2rotm(angles,'XYZ');
if reverse
    rotm = rotm^-1;
end

rotPoints = zeros(size(points));
for i = 1:size(points,1)
    rotPoints(i,:) = (rotm*points(i,:)')';
end