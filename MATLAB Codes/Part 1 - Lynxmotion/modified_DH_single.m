% File:   modified_DH_single.m
% Author:   Jinhang Zhu
% Date:   25 Oct. 2019
% Email:  lm19073@bristol.ac.uk

function FK_mat_single = modified_DH_single(DH_param_single)
% This function calculate the homogeneous transformation from frame i-1 to the ajoint frame i.
%   DH parameters in single transformation.
alpha_im1 = DH_param_single(1,1);   %   alpha i-1   m for minus
a_im1 = DH_param_single(1,2);       %   a i-1
theta_i = DH_param_single(1,3);     %   theta i
d_i = DH_param_single(1,4);         %   d i

%   Single homogeneous transformation from frame i-1 to the ajoint frame i.
T_im1_i = zeros(4,4);
T_im1_i(1,:) = [cosd(theta_i), -sind(theta_i), 0, a_im1];
T_im1_i(2,:) = [sind(theta_i)*cosd(alpha_im1), cosd(theta_i)*cosd(alpha_im1), -sind(alpha_im1), -d_i*sind(alpha_im1)];
T_im1_i(3,:) = [sind(theta_i)*sind(alpha_im1), cosd(theta_i)*sind(alpha_im1), cosd(alpha_im1), d_i*cosd(alpha_im1)];
T_im1_i(4,:) = [0, 0, 0, 1];
FK_mat_single = T_im1_i;
end