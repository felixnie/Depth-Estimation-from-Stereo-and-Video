% Part 2 Depth from Rectified Stereo Images
%
% In this part, a pair of rectified images are used to generate a disparity
% map. The disparity map is then denoised by MRF using multi-label graph
% cut.
%
% Note:
% The weights of the directed graph are 1.
% This code takes 20 ~ 40s to finish.
%
% Tunable variables:
% DISPARITY, LAMBDA, LABELCOST style, DIST style, THRESHOLD, DIRECTION
%

clear all
% close all
% clc
tic
%% load image
img1 = imread('im2.png');
img2 = imread('im6.png');
[H, W] = size(img1, 1:2);

img1 = double(img1);
img2 = double(img2);

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

% regularization of prior term 
lambda = 10;

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
tic
disp('GCMex running. Takes 20 ~ 40 seconds to finish.')
[labels, ~, ~] = GCMex(class, single(unary), ...
                       pairwise, single(labelcost * lambda), 1);
toc

% show original image in RGB
figure
subplot(1,2,1)
imshow(uint8(imgL))
title('Original Image (L)')
subplot(1,2,2)
imshow(uint8(imgR))
title('Original Image (R)')

% show disparity map
figure
[m, i] = min(unary);
disparity_map = reshape(i, W, H)';
imshow(uint8(disparity_map), [min(disparity), max(disparity)])
title(['Original Disparity Map (' direction ')'])

% show result
result = reshape(labels, W, H)';
figure
imshow(uint8(result), [min(disparity), max(disparity)]);
title(['Denoised Disparity Map (Unweighted graph, λ = ' num2str(lambda) ')'])