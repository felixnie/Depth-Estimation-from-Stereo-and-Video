% Part 5 Initialization - Advanced
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
img1 = imread(sprintf('../Road/src/test%04d.jpg',0));
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
img1_coord_homo = [col; row; ones(1, H*W)];

for img1_idx = 0:140
    % read img1
    img1 = imread(['../Road/src/test',sprintf('%04d',img1_idx),'.jpg']);
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
    
    % parallel version
    % to use parallel pool please comment out the preview output part
    parfor idx = 1:length(img2_idx_range)
    
    % serial version
    % for idx = 1:length(img2_idx_range)
    
        img2_idx = img2_idx_range(idx);
        disp(['Processing ' num2str(img1_idx) ' - ' num2str(img2_idx) '.'])
        % read img2
        img2 = imread(sprintf('../Road/src/test%04d.jpg',img2_idx));
        img2 = double(img2);
        [K2, R2, T2] = get_camera_matrices(img2_idx, M);
        
        % data term - set unary term
        % find the pixels on the epipolar line in the other image
        pc = zeros(length(disparity), H*W);
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

            pc(i, :) = sigma_c ./ (sigma_c + sqrt(sum((img1_pixel-img2_pixel).^2, 2)));
        end
        
        unary = unary + pc;
        
%         % view pc - for analyzation
%         unary_tmp = 1 - pc ./ max(pc);
%         [~, index] = min(unary_tmp); 
%         class = index - 1;
%         dmap = reshape(class, [W, H])';
%         imwrite(mat2gray(dmap), sprintf('pc%04d%04d.jpg', img1_idx, img2_idx));
%         figure
%         imshow(uint8(dmap), [min(class), max(class)])
%         title(['pc' sprintf('%04d%04d.jpg', img1_idx, img2_idx)])

%         % view unary - for analyzation
%         unary_tmp = 1 - unary ./ max(unary);
%         [~, index] = min(unary_tmp); 
%         class = index - 1;
%         dmap = reshape(class, [W, H])';
%         imwrite(mat2gray(dmap), sprintf('unary%04d%04d.jpg', img1_idx, img2_idx));
%         figure
%         imshow(uint8(dmap), [min(class), max(class)])
%         title(['unary' sprintf('%04d%04d.jpg', img1_idx, img2_idx)])
    end
    toc
    
    % data term - set unary term - normalization and subtraction
    unary = 1 - unary ./ max(unary);
    [~, index] = min(unary); 
    class = index - 1;
    
    % run graph cut
    disp('GCMex running.')
    tic
    addpath('../GCMex')
    [labels, ~, ~] = GCMex(class, single(unary), pairwise, single(labelcost),1);
    toc
    
    save(sprintf('./init/init%04d.mat', img1_idx), 'labels')
    result = reshape(labels,W,H)';
    imwrite(mat2gray(result), sprintf('./init/init%04d.jpg', img1_idx));
    
%     % show disparity map
%     figure
%     dmap = reshape(class, W, H)';
%     imshow(uint8(dmap), [0, length(disparity)])
%     title('Original Disparity Map')
% 
%     % show result
%     result = reshape(labels,W,H)';
% 	  figure
%     imshow(uint8(result), [min(min(result)), max(max(result))]);
%     title('Denoised Disparity Map')
end