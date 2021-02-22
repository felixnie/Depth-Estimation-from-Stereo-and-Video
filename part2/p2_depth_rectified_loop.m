% Part 2 Depth from Rectified Stereo Images - Loop Version
%
% In this part, a pair of rectified images are used to generate a disparity
% map. The disparity map is then denoised by MRF using multi-label graph
% cut.
%
% Note:
% The weights of the directed graph are 1.
%
% Tunable variables:
% DISPARITY, LAMBDA, LABELCOST style, DIST style, THRESHOLD, DIRECTION
%

clear all
close all
clc
tic
%% load image
img0 = imread('depth.png'); % ground truth
img1 = imread('im2.png');
img2 = imread('im6.png');

[H, W] = size(img1, 1:2);

img1 = double(img1);
img2 = double(img2);

img0 = imresize(img0, [H, W]); % resized uint8 matrix
img0 = rgb2gray(img0); % to grayscale image

% select image order
imgR = img1;
imgL = img2;
% imgR: in which objects are on the right in general.
% imgL: in which objects are on the left in general.

% set disparity matching direction
direction = 'RIGHT-LEFT';
% direction = 'LEFT-RIGHT';
% Please select one of them from above.
% RIGHT-LEFT: disparity searching direction is from Right to Left.
% LEFT-RIGHT: disparity searching direction is from Left to Right.

%% set parameters (class and labelcost)

% set disparity range: disparity = d_min:d_max
disparity = 1:60;
% Change d_max only. This should be less than image width.

% set class
class = ones(1, H*W);

% set labelcost (prior term)
[a, b] = meshgrid(disparity);
labelcost = log(((a - b).^2)./2 + 1); % log distance
% labelcost = abs(a - b); % Euclidean distance (L2)
% This can also be done using function pdist2.

% set threshold for labelcost
% threshold = max(labelcost(1, :)) / 20;
% labelcost = min(labelcost, threshold);


%% calculate (pixel-wise) disparity
unary = zeros(length(disparity), H*W);

for d = disparity
    % fetch the left part of imgL
    imgL_left = imgL(:, 1:end-d, :);
    
    % fetch the right part of imgR
    imgR_right = imgR(:, d+1:end, :);
    
    % calculate the distance
    % dist = sum(abs(imgL_left - imgR_right), 3); % L1 distance
    dist = sqrt(sum((imgL_left - imgR_right).^2, 3)); % L2 distance
    
    % padding with maximum depth when there is no value to compare
    % max_depth = 255 * 3; % L1 distance
    max_depth = sqrt(255 ^ 2 * 3); % L2 distance
    if strcmp(direction, 'RIGHT-LEFT')
        dist_filled = [ones(H, d) * max_depth, dist]';
    elseif strcmp(direction, 'LEFT-RIGHT')
        dist_filled = [dist, ones(H, d) * max_depth]';
    end
    % vectorize
    unary(d, :) = dist_filled(:)';
end

%% set parameters (unary and pairwise)

% set unary (data term)
% Already computed.

% set upper bound for intensity level difference when computing data cost
% cutoff = 30;
% unary = min(unary, cutoff);

% set pairwise (graph structure)
pairwise = construct_graph(H, W);

% run graph cut
addpath('..\GCMex')

lambda = [1, 5, 10, 15, 20, 25, 30, 40, 50, 100, 200, 1000];
MSE = zeros(1, length(lambda));
for i = 1:length(lambda)
    disp(['GCMex running, λ = ' num2str(lambda(i)) '.'])
    tic
    [labels, ~, ~] = GCMex(class, single(unary), ...
                           pairwise, single(labelcost * lambda(i)), 1);
    toc

    % show result
    result = reshape(labels, W, H)';
    % normalization
    result = result * 255 / (max(disparity) - min(disparity));
    result = uint8(result);
    figure
    imshow(result)
    title(['Denoised Disparity Map (Unweighted graph, λ = ', ...
        num2str(lambda(i)), ')'])

    % compute MSE
    MSE(i) = immse(img0, result);
end

figure
plot(lambda, MSE)
title('MSE vs λ')

figure
imshow(img0)
title('Ground Truth')