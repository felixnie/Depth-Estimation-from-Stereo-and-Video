function img = my_feature(img)
% The MATLAB function impixel cannot handle those 3-D arrays that have
% a number of channels that more than 3.
%
% MY_IMPIXEL does the same thing but can return the vectors that are longer
% than 3.
%

[H, W, C] = size(array);
N = length(row);
pixel = NaN(N, C);

for i = 1:N
    r = row(i);
    c = col(i);
    if r < 0 || r > H || c < 0 || c > W
        continue % return NaN
    end
    p = array(r, c, :);
    p = p(:)';
    pixel(i, :) = p;
end

