function pairwise = construct_graph(H, W)
% This function is used to construct a corresponding directed graph of an
% image of H * W. It's faster than the original version in the example code
% of GCMex, since it avoids unnecessary operation on sparse matrices.
%
% The weights are initialized to 1.
%

% my version
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

% weights
weights = ones(length(source), 1);
pairwise = sparse(source, target, weights);


% % third version
% %
% row_wise = sparse(ones(W*(H-1), 1)); % row-wise
% col_wise = sparse(repmat([ones(W-1, 1); 0], H, 1)); % column-wise
% col_wise = col_wise(1:end-1);
% pairwise = diag(row_wise, W) + diag(col_wise, 1);
% pairwise = pairwise + pairwise';

% % second version
% %
% i = [];
% j = [];
% v = [];
% for row = 0:H-1
%     for col = 0:W-1      
%         if row+1 < H
%             i = [i; row*W+col+1];
%             j = [j; 1+col+(row+1)*W];
%             v = [v; 1];
%         end
%         if row-1 >= 0
%             i = [i; row*W+col+1];
%             j = [j; 1+col+(row-1)*W];
%             v = [v; 1];
%         end
%         if col+1 < W
%             i = [i; row*W+col+1];
%             j = [j; 1+(col+1)+row*W];
%             v = [v; 1];
%         end
%         if col-1 >= 0
%             i = [i; row*W+col+1];
%             j = [j; 1+(col-1)+row*W];
%             v = [v; 1];
%         end
%     end
% end
% 
% pairwise = sparse(i,j,v);

% % original version
% % This is the method given by the example of GCMex
% %
% pairwise = sparse(H*W, H*W);
% for row = 0:H-1
%     for col = 0:W-1      
%         if row+1 < H,   pairwise(row*W+col+1, 1+col+(row+1)*W) = 1; end
%         if row-1 >= 0,  pairwise(row*W+col+1, 1+col+(row-1)*W) = 1; end
%         if col+1 < W,   pairwise(row*W+col+1, 1+(col+1)+row*W) = 1; end
%         if col-1 >= 0,  pairwise(row*W+col+1, 1+(col-1)+row*W) = 1; end
%     end
% end