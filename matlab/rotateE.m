function rotPoints = rotateE(mat,points,reverse)
% mat = -mat; % set negative angles to make it work
% rotm = eul2rotm(mat,'XYZ'); % get the rotation matrix
% if reverse % if its necessary to go back to the global
%     rotm = rotm^-1; % reverse
% end
% 
% rotPoints = zeros(size(points));
% for i = 1:size(points,1) % for each point
%     rotPoints(i,:) = (rotm*points(i,:)')'; % convert points
% end
if reverse
    mat = mat';
end
rotPoints = zeros(size(points));
for i = 1:size(points,1) % for each point
    rotPoints(i,:) = (mat*points(i,:)')'; % convert points
end