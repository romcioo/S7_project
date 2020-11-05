function rect = getRect(T)
rect = zeros(1,4);
if T(2,1) == 1
    rect(1:2) = [2 1];
else
    rect(1:2) = [1 2];
end
if T(1,3) == 3
    rect(3:4) = [4 3];
else
    rect(3:4) = [3 4];
end