function rotPoints = rotateE(angles,points,reverse)
% disp(angles)
% disp(points)
angles = -angles;
rotm = eul2rotm(angles,'XYZ');
if reverse
    rotm = rotm^-1;
end
% assignin('base','rotm',rotm)

rotPoints = zeros(size(points));
for i = 1:size(points,1)
    rotPoints(i,:) = (rotm*points(i,:)')';
end