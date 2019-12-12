% File:   modified_DH_whole.m
% Author:   Jinhang Zhu
% Date:   25 Oct. 2019
% Email:  lm19073@bristol.ac.uk

function FK_mat_whole = modified_DH_whole(DH_param_whole)
% This function calculate the homogeneous transformation from base to the end-effector.
%   Get the number of transformations, i.e. frames
DH_param_size = size(DH_param_whole);
n_trans = DH_param_size(1,1);
%   Calculate the whole homogeneous transformation matrix via a loop.
T_0_n = eye(4);

for i = 1:n_trans
   T_im1_i = modified_DH_single(DH_param_whole(i,:));   % Single homogeneous matrix from i-1 to i
   T_0_n = T_0_n *T_im1_i;
end
FK_mat_whole = T_0_n;
end