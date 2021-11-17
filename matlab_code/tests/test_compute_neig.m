N = 3;
max_neig = 2;
r_comm = 150;
rng(4)
position = rand(3*N,1);


dist2_matrix = zeros(N,N);
M = zeros(N,N);

for agent = 1:N
    for neig = 1:N
        if agent == neig
            dist2_matrix(agent,neig) = 10;
        elseif neig > agent
            dist2 = sum((position(3*(agent-1)+[1 2 3]) - ...
                position(3*(neig-1)+[1 2 3])).^2);
            dist2_matrix(agent,neig) = dist2;
            dist2_matrix(neig,agent) = dist2; % for simmetry
        end
    end
end

sorted_neig = zeros(max_neig,N);

for i = 1:N % for every column, order column
    
    to_sort = ones(N,1);
    dist2_to_sort = dist2_matrix(:,i);
    for j = 1:max_neig % order only the first max_neig
        min_idx = 0;
        min_dist2 = 1e9; % high init value
        for k = 1:N
            if dist2_to_sort(k) < min_dist2 && ...
                    dist2_to_sort(k) < r_comm^2 && ...
                    to_sort(k)
                min_idx = k;
            end
            if dist2_to_sort(k) < min_dist2 && ...
                    dist2_to_sort(k) < r_comm^2 && ...
                    to_sort(k)
                min_dist2 = dist2_to_sort(k);
            end
        end
        sorted_neig(j,i) = min_idx;
        for l = 1:N
            if min_idx == l
                to_sort(l) = 0;
                M(l,i) = 1;
            end
        end
        
    end
end

disp(dist2_matrix)
disp(sorted_neig)