% E.g. can this simple loop:
A = zeros(1,100);
for i = 1:100
    A(i) = 2;
end

% be replaced by a single and more efficient command:
B = ones(1,100) * 2;

% And why write:
for i = 1:10
    for j = 1:10
        stor_1(i,j) = i^2; % MATLAB warning: consider preallocating for speed
    end
end