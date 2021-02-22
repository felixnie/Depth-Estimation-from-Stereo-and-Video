% Part 1 Noise Removal - Loop Version
%

clear all
close all
% clc
tic
% load image
img = imread('bayes_in.jpg');
% img = imresize(img, 0.2);
[H, W] = size(img, 1:2);

img = double(img);

% prepare colormap
blue  = [  0;   0; 255];
beige = [245; 210; 110];
source = blue;  % foreground color
sink = beige;   % background color

img_sink = ones(H,W,3);
img_source = ones(H,W,3);
for i = 1:3
    img_sink(:,:,i) = sink(i) * ones(H,W);
    img_source(:,:,i) = source(i) * ones(H,W);
end
diff_sink = sum(abs(img - img_sink), 3) / 3;
diff_source = sum(abs(img - img_source), 3) / 3;

class = diff_source < diff_sink;
img_binary = reshape(class, H, W);

% set class, unary, pairwise
class = zeros(H*W, 1);
% class = class';
% class = double(class(:));

diff_sink = diff_sink';
diff_source = diff_source';
unary = [diff_sink(:) diff_source(:)]'; % construct data term
pairwise = construct_graph(H, W);

% % show original image in RGB
% figure
% subplot(1,2,2)
% imshow(uint8(img))
% title('Original Image in RGB')
% 
% % show original image in binary
% subplot(1,2,1)
% imshow(uint8(img_binary), [0 1])
% title('Original Image in Binary')

% set lambda and labelcost
lambda_list = [0, 1, 10, 50, 100, 200, 500, 1000, 2000] ;
for lambda = lambda_list

    labelcost = [0, 1; 1, 0] * lambda;

    % run graph cut
    addpath('..\GCMex')
    [labels, E, Eafter] = GCMex(class, single(unary), pairwise, single(labelcost), 0);
    disp(['Energy in CLASS: ' num2str(E) '. Energy in LABELS: ' num2str(Eafter) '.'])

    img_filtered = reshape(labels, W, H)';
    figure
%     subplot(1,2,1)
%     imshow(uint8(img_filtered), [0 1])
%     title(['Filtered Image λ = ' num2str(lambda)])
% 
%     subplot(1,2,2)
    img_rgb = img_filtered .* img_source + (1 - img_filtered) .* img_sink;
    imshow(uint8(img_rgb))
    title(['Filtered Image λ = ' num2str(lambda)])

    toc
end