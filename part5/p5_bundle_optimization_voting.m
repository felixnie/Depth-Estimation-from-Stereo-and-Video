% Part 4 Bundle Optimization
%

clear all
close all
clc
tic
%% load image

% load cam matrix
fileID = fopen('../Road/cameras.txt');
C = textscan(fileID,'%f %f %f');
fclose(fileID);
M = cell2mat(C);
M(1,:) = [];

inconsistency_list = zeros(1, 141);
inconsistency_weight = zeros(1, 141);

% set disparity
dmin = 0;
dmax = 0.01;
step = 0.0001;
disparity = dmin:step:dmax;

% prior term - set labelcost
eta = 0.05 * (max(disparity) - min(disparity));
[a, b] = meshgrid(disparity);
rho = min(abs(a - b), eta);
labelcost = rho;

% number of imgs used
num_neighbour = 30;

% img size
img1 = imread(sprintf('../Road/src/test%04d.jpg', 0));
[H, W] = size(img1, 1:2);

% parameters for prior term - set pairwise (smoothness weight)
epsilon = 50;
N_x = 4 * ones(H, W);
N_x([1, end], :) = N_x([1, end], :) - 1;
N_x(:, [1, end]) = N_x(:, [1, end]) - 1;
N_x = N_x'; N_x = N_x(:)';
w_s = 2 ./ (disparity(end) - disparity(1));

% parameters for data term - set unary term
sigma_c = 10;
row = repmat(1:H, W, 1);
row = row(:)';
col = repmat((1:W)', 1, H);
col = col(:)';
img1_coord = [col; row];
img1_coord_homo = [col; row; ones(1, H*W)];

for img1_idx = 0:140

    % read img1
    img1 = imread(sprintf('../Road/src/test%04d.jpg', img1_idx));
    img1 = double(img1);
    [K1, R1, T1] = get_camera_matrices(img1_idx, M);
    
    % prior term - set pairwise (smoothness weight)
    graph = construct_graph_weighted(H, W, img1, epsilon);
    [source, target, weight] = find(graph);
    denominator = sum(graph);
    u_lambda = N_x ./ denominator;
    lambda = w_s .* u_lambda(source) .* weight';
    pairwise = sparse(source, target, lambda);
    
    % get pixel values from the image
    img1_pixel = impixel(img1, col, row);
    
    % set img2 index range
    img2_idx_range = img1_idx - num_neighbour/2 : img1_idx + num_neighbour/2;
    img2_idx_range(img2_idx_range < 0 | img2_idx_range == img1_idx | img2_idx_range > 140) = [];
    
    % initialization for unary term
    unary = zeros(length(disparity), H*W);
    tic
    
    inconsistency = zeros(1, length(img2_idx_range));
    % parallel version
    % to use parallel pool please comment out the preview output part
    % parfor idx = 1:length(img2_idx_range)
    
    % serial version
    for idx = 1:length(img2_idx_range)
    
        img2_idx = img2_idx_range(idx);
        disp(['Processing ' num2str(img1_idx) ' - ' num2str(img2_idx) '.'])
        % read img2
        img2 = imread(sprintf('../Road/src/test%04d.jpg', img2_idx));
        img2 = double(img2);
        [K2, R2, T2] = get_camera_matrices(img2_idx, M);

        % TEST
        img1_labels = load(sprintf('./init/init%04d.mat', img1_idx)).labels;
        img2_labels = load(sprintf('./init/init%04d.mat', img2_idx)).labels;
        img1_labels_map = reshape(img1_labels, W, H)';
        img2_labels_map = reshape(img2_labels, W, H)';
        
%         img1_project_homo = K1 * R1' * R2 / K2 * img2_project_homo + ...
%                 K1 * R1' * (T2 - T1)' .* disparity_img2(img2_coord_idx);
        img2_coord_homo = K2 * R2' * R1 / K1 * img1_coord_homo + ...
                K2 * R2' * (T1 - T2)' .* disparity(img1_labels+1);
        % normalization and turn into inhomogeneous system
        img2_coord = img2_coord_homo ./ img2_coord_homo(3, :);
        img2_coord = round(img2_coord(1:2, :));
        
        img2_labels_project = my_impixel(img2_labels_map, img2_coord(1, :), img2_coord(2, :));   
        
        img2_labels_project = reshape(img2_labels_project, W, H)';
        outliers = isnan(img2_labels_project);
        ratio_inliers = (H*W - sum(sum(outliers))) / (H*W);

        img2_labels_project(outliers) = 0; % any number is fine
        
%         inconsistency_map = sqrt((img2_labels_project - img1_labels_map).^2);
        inconsistency_map = (img2_labels_project - img1_labels_map).^2;

        inconsistency_map(outliers) = 0;
%         figure
%         imshow(mat2gray(inconsistency_map));
%         title(num2str([img1_idx img2_idx]))
        inconsistency(idx) = sum(sum(inconsistency_map));
        inconsistency_list(img2_idx + 1) = inconsistency_list(img2_idx + 1) + ...
            ratio_inliers * sum(sum(inconsistency_map));
        inconsistency_weight(img1_idx + 1) = inconsistency_weight(img1_idx + 1) + ...
            ratio_inliers * sum(sum(inconsistency_map));
        imwrite(mat2gray(inconsistency_map), sprintf('./inconsistency_map/%04d%04d.jpg', img1_idx, img2_idx));
        
    end
    toc

    fig = figure("Visible",false);
    stem(img2_idx_range, inconsistency);
    saveas(fig, sprintf('./inconsistency_figure/inco%04d.jpg', img1_idx))
    
end
close all

figure
stem(0:140, inconsistency_list)
hold on
plot(0:140, mean(inconsistency_list) * ones(1,141), '-r')
title('Inconsistency Votes Received')

figure
stem(0:140, inconsistency_weight)
hold on
plot(0:140, mean(inconsistency_weight) * ones(1,141), '-r')
title('Inconsistency Votes Sent')