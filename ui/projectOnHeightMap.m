function pts = projectOnHeightMap(pts, mesh)
% list of points to project
% mesh is a heightmap

V = mesh.Points;
E = edges(mesh);

E_lines = [V(E(:,1),1:2) V(E(:,2),1:2)];
p_lines = [pts(1:end-1,1:2) pts(2:end,1:2)];

out = lineSegmentIntersect(p_lines, E_lines);

for iseg = size(pts,1)-1 : -1 : 1
    idx = find(out.intAdjacencyMatrix(iseg,:));
    
    dist = out.intNormalizedDistance(iseg,idx);
    [~,sorted_idx] = sort(dist);
    
    iE = idx(sorted_idx);
    pts_to_add = zeros(length(sorted_idx),3);
    pts_to_add(:,1) = out.intMatrixX(iseg,iE);
    pts_to_add(:,2) = out.intMatrixY(iseg,iE);

    Ve = reshape(V(E(iE,:),:),[],2,3);
    pts_to_add(:,3) = Ve(:,1,3) + dist(sorted_idx)' .* (Ve(:,2,3)-Ve(:,1,3));
    
    pts = [pts(1:iseg,:); pts_to_add; pts(iseg+1:end,:)];
end

end