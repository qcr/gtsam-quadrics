function Tmn = TvecMat(m,n)
% from: https://au.mathworks.com/matlabcentral/fileexchange/26781-vectorized-transpose-matrix
% Builds the orthogonal transpose vectorization matrix of an m by n matrix.
% Tmn = TvecMat(m,n)
%
% Tmn is an orthogonal permutation matrix called the "vectorized transpose
% matrix" of an m#n matrix. For example, if A is m#n then Tmn*vec(A) =
% vec(A.'), where vec(A) is the "vectorization of A" defined as the column
% vector formed by concatenating all the columns of A. vec(A) = A(:).
%
% Other uses are in the tensor product, where B(x)A = Tpm( A(x)B )Tnq where
% A is m#n and B is p#q and (x) denotes the tensor product.
%
%
% %%% ZCD Feb 2010 %%%
%
d = m*n;
Tmn = zeros(d,d);
i = 1:d;
rI = 1+m.*(i-1)-(m*n-1).*floor((i-1)./n);
I1s = sub2ind([d d],rI,1:d);
Tmn(I1s) = 1;
Tmn = Tmn';