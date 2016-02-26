
function [minv]= matrix3by3_inverse(m)

% computes the inverse of a matrix m
det = m(1, 1) * (m(2, 2) * m(3, 3) - m(3, 2) * m(2, 3)) - m(1, 2) * (m(2, 1) * m(3, 3) - m(2, 3) * m(3, 1)) + m(1, 3) * (m(2, 1) * m(3, 2) - m(2, 2) * m(3, 1));

invdet = 1 / det;

minv = zeros(3,3); % inverse of matrix m
minv(1, 1) = (m(2, 2) * m(3, 3) - m(3, 2) * m(2, 3)) * invdet;
minv(1, 2) = (m(1, 3) * m(3, 2) - m(1, 2) * m(3, 3)) * invdet;
minv(1, 3) = (m(1, 2) * m(2, 3) - m(1, 3) * m(2, 2)) * invdet;
minv(2, 1) = (m(2, 3) * m(3, 1) - m(2, 1) * m(3, 3)) * invdet;
minv(2, 2) = (m(1, 1) * m(3, 3) - m(1, 3) * m(3, 1)) * invdet;
minv(2, 3) = (m(2, 1) * m(1, 3) - m(1, 1) * m(2, 3)) * invdet;
minv(3, 1) = (m(2, 1) * m(3, 2) - m(3, 1) * m(2, 2)) * invdet;
minv(3, 2) = (m(3, 1) * m(1, 2) - m(1, 1) * m(3, 2)) * invdet;
minv(3, 3) = (m(1, 1) * m(2, 2) - m(2, 1) * m(1, 2)) * invdet;

