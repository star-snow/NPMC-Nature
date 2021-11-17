function [M, sorted_neig] =  compute_closest_neighbors(position, r_comm, max_neig)
% compute_neighborhood_casadi - Build adjacency matrix
%
% Adjacency matrix M(i,j): 
% NxN matrix filled with 0/1: 0 means i is not neig of j and 1 means it is.
% Column j contains the information about neighbours i of agent j
% It is asymmetric in case of limited fov, symmetric otherwise.
% Note: agent j is not neig of itself (zeros on diagonal).
%
% Input:
%   position: vector of positions of the agents [p1 p2 ...], p1 is 3x1
%   r_comm: radius of communication of every agent
%   max_neig: maximum number of neighbours considered
%
% Output:
%   M: adjacency matrix (NxN)
%   sorted_neig: matrix of sorted neig (max_neig,N)
    
    import casadi.*

    N = length(position)/3;
    dist2_matrix = SX.sym('D',N,N);
    
    M = SX.zeros(N,N);
    
    for agent = 1:N
        for neig = 1:N
            
            if agent == neig
                dist2_matrix(agent,neig) = 1e10;
                
            elseif neig > agent
                dist2 = sum((position(3*(agent-1)+[1 2 3]) - ...
                    position(3*(neig-1)+[1 2 3])).^2);
                dist2_matrix(agent,neig) = dist2;
                dist2_matrix(neig,agent) = dist2; % for simmetry
                
            end
        end
    end
    
    sorted_neig = SX.zeros(max_neig,N);
    for i = 1:N % for every column, order column
        
        to_sort = SX.ones(N,1);
        dist2_to_sort = dist2_matrix(:,i);
        
        for j = 1:max_neig % order only the first max_neig
            min_idx = 0;
            min_dist2 = SX(1e9); % high init value
            
            for k = 1:N
                min_idx = if_else(dist2_to_sort(k) < min_dist2 & ...
                    dist2_to_sort(k) < r_comm^2 & ...
                    to_sort(k), ...
                    k, min_idx);
                min_dist2 = if_else(dist2_to_sort(k) < min_dist2 & ...
                    dist2_to_sort(k) < r_comm^2 & ...
                    to_sort(k), ...
                    dist2_to_sort(k), min_dist2);
            end
            
            sorted_neig(j,i) = min_idx;
            
            for l = 1:N
                to_sort(l) = if_else(min_idx == l , ...
                    0, to_sort(l));
                M(l,i) = if_else(min_idx == l, ...
                    1, M(l,i));
            end
            
        end
    end
    
end