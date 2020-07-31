function pt = findIntersection(line, mesh)
% line represented as 2 points on the line
% mesh is a triangulation

V = mesh.Points;
CL = mesh.ConnectivityList;
N = mesh.faceNormal; % normal vectors to mesh faces

verts = zeros(size(CL,1),size(V,2),3);
verts(:,:,1) = V(CL(:,1),:);
verts(:,:,2) = V(CL(:,2),:);
verts(:,:,3) = V(CL(:,3),:);

dist = dot(verts(:,:,1)-line(1,:),N,2)./((line(2,:)-line(1,:))*N')';
P0 = line(1,:) + dist*(line(2,:)-line(1,:));
tokeep = dot(cross(verts(:,:,2)-verts(:,:,1),P0-verts(:,:,1),2),N,2)>=0 & ...
    dot(cross(verts(:,:,3)-verts(:,:,2),P0-verts(:,:,2),2),N,2)>=0 & ...
    dot(cross(verts(:,:,1)-verts(:,:,3),P0-verts(:,:,3),2),N,2)>=0;
dist(~tokeep) = Inf;
[~,i] = min(abs(dist));
pt = P0(i,:);

end