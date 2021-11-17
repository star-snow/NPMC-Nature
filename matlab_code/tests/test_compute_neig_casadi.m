import casadi.*

N = 3;
max_neig = 2;
r_comm = 150;
rng(4)
position = rand(3*N,1);

pos_matrix = reshape(position, 3, []);
dist = pdist(pos_matrix');
dist_matrix = squareform(dist);

[M, sorted_neig] =  compute_closest_neighbors(position, r_comm, max_neig);

disp(dist_matrix)
disp(M)
disp(sorted_neig)