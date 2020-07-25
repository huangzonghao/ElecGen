function [edges, grp_assign] = reduceEdges(TR, threshold)
edges = zeros(0,2);

T = TR.ConnectivityList;
normals = faceNormal(TR);
neigh = neighbors(TR);

grp_assign = 1:size(T,1);

for i = 1:size(normals)
    for j = 1:size(neigh,2)
        if (grp_assign(i) ~= grp_assign(neigh(i,j)))
            if (acos(dot(normals(i,:), normals(neigh(i,j),:))) > threshold)
                overlap = intersect(T(i,:),T(neigh(i,j),:))';
                if length(overlap)==2
                    edges(end+1,:) = overlap;
                end
            else
                grp_assign(grp_assign==grp_assign(neigh(i,j))) = grp_assign(i);
            end
        end
    end
end

[~,~,grp_assign] = unique(grp_assign);

end