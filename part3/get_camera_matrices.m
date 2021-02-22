function [K1, R1, T1, K2, R2, T2] = get_camera_matrices()

fileID = fopen('cameras.txt');
C = textscan(fileID,'%f %f %f');
fclose(fileID);

M = cell2mat(C);

K1 = M(1:3  , :);
R1 = M(4:6  , :);
T1 = M(7    , :);

K2 = M(8:10 , :);
R2 = M(11:13, :);
T2 = M(14   , :);