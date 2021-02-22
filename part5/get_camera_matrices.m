function [K, R, T] = get_camera_matrices(idx, M)

K = M(idx * 7 + (1:3)  , :);
R = M(idx * 7 + (4:6)  , :);
T = M(idx * 7 + (7  )  , :);