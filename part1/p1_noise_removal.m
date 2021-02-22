% Part 1 Noise Removal
%
% This code utilizes MATLAB's parallel matrix computation feature to
% accelerate the calculation. It's much faster than GCMex_test when
% constructing graph structure and calculating data terms. The running time
% is shown in the command window.
%
% p1_noise_removal_loop is the looping version that iterate over each
% lambda and find the optimal setting.
%

clear all
close all
% clc
tic

%% load image
img = imread('bayes_in.jpg');
% img = imresize(img, 0.2);
[H, W] = size(img, 1:2);

img = double(img);

%% prepare colormap
blue  = [  0;   0; 255];
beige = [245; 210; 110];
source = blue;  % foreground color
sink = beige;   % background color

%% binarize original image for display
img_sink = ones(H,W,3);
img_source = ones(H,W,3);
for i = 1:3
    img_sink(:,:,i) = sink(i) * ones(H,W);
    img_source(:,:,i) = source(i) * ones(H,W);
end
diff_sink = sum(abs(img - img_sink), 3) / 3;
diff_source = sum(abs(img - img_source), 3) / 3;

img_binary = diff_source < diff_sink;
img_binary = reshape(img_binary, H, W);

%% set parameters
% set class
class = zeros(H*W, 1);
% class = class';
% class = double(class(:));

% set unary (data term)
diff_sink = diff_sink';
diff_source = diff_source';
unary = [diff_sink(:) diff_source(:)]';

% set pairwise (graph structure)
pairwise = construct_graph(H, W);

% set lambda and labelcost (prior term)
lambda = 190;
labelcost = [0, 1; 1, 0] * lambda;

%% run graph cut
addpath('..\GCMex')
[labels, E, Eafter] = GCMex(class, single(unary), pairwise, single(labelcost), 0);
disp(['Energy in CLASS: ' num2str(E) '. Energy in LABELS: ' num2str(Eafter) '.'])

%% display
% show original image in RGB
figure
imshow(uint8(img))
title('Original Image in RGB')

% show original image in binary
figure
imshow(uint8(img_binary), [0 1])
title('Original Image in Binary')

% show filtered image in binary
img_filtered = reshape(labels, W, H)';
figure
imshow(uint8(img_filtered), [0 1])
title('Filtered Image in Binary')

% show filtered image in RGB
img_rgb = img_filtered .* img_source + (1 - img_filtered) .* img_sink;
figure
imshow(uint8(img_rgb))
title('Filtered Image in RGB')

toc