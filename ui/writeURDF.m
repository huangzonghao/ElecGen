function writeURDF(filename, name, links, joints, functionname, functiontype)

if ~exist('functionname', 'var')
    functionname = arrayfun(@(i)['sensor' num2str(i)], 1:length(links(1).buttonState), 'uniformoutput', false);
else
    functionname = cellfun(@(x)remove_whitespace(x),functionname,'uniformoutput',false);
end

if ~exist('functiontype', 'var')
    functiontype = false(1,length(links(1).buttonState));
end

fid = fopen(filename, 'w');

%% initialize robot
fprintf(fid, '<robot name = "%s">\n\n', name);

%% write all the links
for i = 1:length(links)
    fprintf(fid, links(i).text);
    fprintf(fid, '\n');
end

fprintf(fid, '\n');

%% write all the joints
for i = 1:length(joints)
    fprintf(fid, joints(i).text);
    fprintf(fid, '\n');
end

fprintf(fid, '\n');

ipart = 0;
%% add link motors/sensors
for i = 1:length(links)
    [on_face, has_function] = find(links(i).buttonState);
    for isensor = 1:length(on_face)
        iface = on_face(isensor);
        location = findCentroid(links(i).mesh, find(links(i).face_groups==iface));
        ifunction = has_function(isensor);
        
        % add sensor to the centroid of the selected face
        writeSensor(fid, [functionname{ifunction} '_' num2str(ipart)], links(i), location);
    end
end

%% add joint motors/sensors
for i = 1:length(joints)
    for ifunction = 1:length(joints(i).buttonState)
        if joints(i).buttonState(ifunction)
            ipart = ipart+1;
            switch functiontype(ifunction)
                case 1 % actuator
                    writeActuator(fid, [functionname{ifunction} '_' num2str(ipart)], joints(i));
                case 0 % sensor
                    % sensor is on the child
                    writeSensor(fid, [functionname{ifunction} '_' num2str(ipart)], links(joints(i).parent), joints(i).origin);
            end
        end
    end
end

%% end robot
fprintf(fid, '</robot>');

fclose(fid);

end

function centroid = findCentroid(mesh, faces)
CL = mesh.ConnectivityList;
V = mesh.Points;

centroid = [0 0 0];
for iface = faces(:)'
    midpoint = (V(CL(iface,2),:)-V(CL(iface,3),:))/2 + V(CL(iface,3),:);
    centroid_i = 2/3*(midpoint-V(CL(iface,1),:)) + V(CL(iface,1),:);
    centroid = centroid + centroid_i;
end
centroid = centroid ./ length(faces);

end

function writeActuator(fid, name, joint)
fprintf(fid, '<motor name = "%s">\n', name);
fprintf(fid, '<joint name = "%s"/>\n', joint.name);
% FIXME - actuator is currently on the child
fprintf(fid, '<mass value = "1" xyz = "%f %f %f"/>\n', ...
    joint.origin(1), joint.origin(2), joint.origin(3));
fprintf(fid, '</motor>\n');
end

function writeSensor(fid, name, link, location)
fprintf(fid, '<extra_mass name = "%s">\n', name);
fprintf(fid, '<link name = "%s"/>\n', link.name);
fprintf(fid, '<mass value = "1" xyz = "%f %f %f"/>\n', ...
    location(1), location(2), location(3));
fprintf(fid, '</extra_mass>\n');
end

function str = remove_whitespace(str)
str = erase(str, [" ", "<br>"]);
end