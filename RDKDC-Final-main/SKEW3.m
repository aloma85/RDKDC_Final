function ss_matrix = SKEW3(X)
x1 = X(1, 1);
x2 = X(2, 1);
x3 = X(3, 1);
ss_matrix = [0 -1 * x3 x2; x3 0 -1 * x1; -1 * x2 x1 0];


