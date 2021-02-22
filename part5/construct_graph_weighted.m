function pairwise = construct_graph_weighted(H, W, img, epsilon)
% This function is used to construct a corresponding directed graph of an
% image of H * W.
%
% The graph describes a spatially varying smoothness cost. It's weighted
% with the L2 distance in color space between the adjacent pixels.
%

% index matrix
index = reshape(1:H*W, W, H)'; % row-wise indexing

% up-down connection
up      = index(1:end-1, :);
down    = index(2:end, :);

% left-right connection
left    = index(:, 1:end-1);
right   = index(:, 2:end);

% source and target pairs
source  = [up(:); down(:); left(:); right(:)];
target  = [down(:); up(:); right(:); left(:)];

% vectorize
r = img(:, :, 1)'; r = r(:);
g = img(:, :, 2)'; g = g(:);
b = img(:, :, 3)'; b = b(:);

% weights
weights = (r(source) - r(target)).^2 + ...
          (g(source) - g(target)).^2 + ...
          (b(source) - b(target)).^2;
weights = 1 ./ (sqrt(weights) + epsilon);
% weights = ones(length(source), 1);
pairwise = sparse(source, target, double(weights));