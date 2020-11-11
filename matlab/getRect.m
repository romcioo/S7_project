function rect = getRect(T)
rect = zeros(1,4);
long = reshape(T,[1 6]); % flat the T
mIn = min(long); % get the minimum value (first point)
if T(2,1) == mIn % if the first number of the second triangle is the minimum
    rect(1:2) = [mIn+1 mIn]; % the first two points
else
    rect(1:2) = [mIn mIn+1];
end
if T(1,3) == mIn+2 % if the last number of the first triangle is the the third number
    rect(3:4) = [mIn+3 mIn+2]; % the last two numbers
else
    rect(3:4) = [mIn+2 mIn+3];
end