% Part 3 Depth from Unrectified Stereo Images
%
% In this part, a pair of unrectified images are used to generate a
% disparity map. The disparity map is then denoised by MRF using
% multi-label graph cut.
%
% Note:
% The weights of the directed graph are defined according to the paper.
% This code takes 120 ~ 160s to finish.
%

clear all
close all
clc
tic
%% load image
img1 = imread('test00.jpg');
img2 = imread('test09.jpg');

[H, W] = size(img1, 1:2);

img1 = double(img1);
img2 = double(img2);

% get camera matrices
[K1, R1, T1, K2, R2, T2] = get_camera_matrices();

% set disparity
dmin = 0;
dmax = 0.01;
step = 0.0001;
disparity = dmin:step:dmax;

%% prior term - part 1: set pairwise (smoothness weight)

% epsilon: ε, contrast sensitivity controller.
epsilon = 50;

% construct a graph with weight : 1 / (||It(x) - It(y')|| + ε)
graph = construct_graph_weighted(H, W, img1, epsilon);
[source, target, weight] = find(graph);

% N_x: |N(x)|, num of neighors of pixel x.
% It should be a matrix like this:
%   2 3 3 3 2
%   3 4 4 4 3
%   2 3 3 3 2
N_x = 4 * ones(H, W);
N_x([1, end], :) = N_x([1, end], :) - 1;
N_x(:, [1, end]) = N_x(:, [1, end]) - 1;
N_x = N_x'; N_x = N_x(:)'; % vectorization

% The denominator Σ 1 / (||It(x) - It(y')|| + ε) can be constructed by
% summing the symmetric weight matrix.
denominator = sum(graph);

% u_lambda: u_λ(x), the normalization factor of graph weight λ.
% u_λ(x) = |N(x)| / Σ 1 / (||It(x) - It(y')|| + ε)
% ||It(x) - It(y')|| is the L2 norm of the difference between pixel x and
% its neighbouring pixel y'.
u_lambda = N_x ./ denominator;

% w_s: smoothness-strength controller.
w_s = 10 ./ (disparity(end) - disparity(1));

% Now we can construct the weight for spatially varying graph.
% lambda: λ(x,y), anisotropic weight of the directed graph.
% λ(x,y) = w_s * u_λ(x) / (||It(x) - It(y)|| + ε)
% ||It(x) - It(y)|| is the L2 norm of the difference between pixel x and
% its neighbouring pixel y.
lambda = w_s .* u_lambda(source) .* weight';

% reconstruct and add weight to the graph
pairwise = sparse(source, target, lambda);


%% prior term - part 2: set labelcost

% eta: η, the upper limit of the cost.
eta = 0.05 * (max(disparity) - min(disparity));

% rho: ρ, a robust function of labelling cost.
% ρ(Dt(x), Dt(y)) = min{|Dt(x), Dt(y)|, η}
% |Dt(x), Dt(y)| is the L1 norm of the difference between disparity lavels.
[a, b] = meshgrid(disparity);
rho = min(abs(a - b), eta);

% set labelcost
labelcost = rho;

%% data term - part 1: set unary term
% p_c: measures color similarity between pixel x and the corresponding
% pixel x' (given disparity d) in frame t'. In this case, t' in the other
% image. The similarity measurement gets larger when the color of the
% corresponding pixels gets closer.
%
% p_c(x, d, It, It') = σ_c / (σ_c + ||It(x) - It'(ltt'(x, d))||)
%
% ltt' is the epipolar line. ltt'(x, d) gives the location of x' on frame
% t', also on the epipolar line, with disparity = d.

% sigma_c: σ_c, controls the shape of differentiable robust function.
sigma_c = 10;

unary = zeros(length(disparity), H*W); % initialization

% generate row-wise pixel coordinates
row = repmat(1:H, W, 1);
row = row(:)';
col = repmat((1:W)', 1, H);
col = col(:)';

% img1_coord_homo: x, coordinates in img1 in homogeneous system
img1_coord_homo = [col; row; ones(1, H*W)];
img1_pixel = impixel(img1, col, row);

% Let's find the pixels on the epipolar line in the other image.
for i = 1:length(disparity)
    img2_coord_homo = K2 * R2' * R1 / K1 * img1_coord_homo + ...
        disparity(i) * K2 * R2' * (T1 - T2)';
    % normalization and turn into inhomogeneous system
    img2_coord = img2_coord_homo ./ img2_coord_homo(3, :);
    img2_coord = round(img2_coord(1:2, :));
    
    % get pixel values from the other image
    img2_pixel = impixel(img2, img2_coord(1, :), img2_coord(2, :));
    
    % check the coordinates if they exceed the image size
    outliers = isnan(img2_pixel);
    img2_pixel(outliers) = 0;
    
    disparity_vector = sigma_c ./ (sigma_c + sqrt(sum((img1_pixel-img2_pixel).^2, 2)));
    unary(i, :) = disparity_vector';
end

% normalization and subtraction
unary = 1 - unary ./ max(unary);

[~, index] = min(unary); 
class = index - 1;

%% run graph cut
disp('GCMex running.')
tic
addpath('..\GCMex')
[labels, ~, ~] = GCMex(class, single(unary), pairwise, single(labelcost),1);
toc

%% display

% show original image in RGB
figure
% subplot(1,2,1)
imshow(uint8(img1))
title('Original Image (L)')

figure
% subplot(1,2,2)
imshow(uint8(img2))
title('Original Image (R)')

% show disparity map
figure
disparity_map = reshape(class, W, H)';
imshow(uint8(disparity_map), [0, length(disparity)])
title('Original Disparity Map')

% show result
result = reshape(labels,W,H)';
figure
imshow(uint8(result), [min(min(result)), max(max(result))]);
title('Denoised Disparity Map')