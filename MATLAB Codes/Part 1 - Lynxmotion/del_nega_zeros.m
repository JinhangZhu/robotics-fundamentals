function dstMat = del_nega_zeros(srcMat)
% Convert negative zeros to zeros
thres_nega_zero = -1e-10;
srcMat(srcMat<0 & srcMat>thres_nega_zero)=0;
dstMat = srcMat;

end