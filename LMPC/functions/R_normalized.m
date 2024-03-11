function R = R_normalized(R)
[U,S,V] = svd(R);
R = U*V';
end