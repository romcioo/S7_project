function rect = getRect(T)
rect = zeros(1,4);
long = reshape(T,[1 6]); % flat the T
mIn = min(long); % get the minimum value (first point)
if T(2,1) == mIn % if the first number is the minimum
    rect(1:2) = [mIn+1 mIn]; % the first two points
else
    rect(1:2) = [mIn mIn+1];
end
if T(1,3) == mIn+2
    rect(3:4) = [mIn+3 mIn+2];
else
    rect(3:4) = [mIn+2 mIn+3];
end