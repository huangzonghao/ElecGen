function F = reduceEdges(TR, threshold)
F = zeros(2,0);

T = TR.ConnectivityList;
normals = faceNormal(TR);
neigh = neighbors(TR);

for i = 1:size(normals)
    for j = 1:size(neigh,2)
        if (acos(dot(normals(i,:), normals(neigh(i,j),:))) > threshold)
            overlap = intersect(T(i,:),T(neigh(i,j),:))';
            if length(overlap)==2
                F(:,end+1) = overlap;
            end
        end
    end
end

end