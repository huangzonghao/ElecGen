% loadURDF
%
% Read in the URDF file
%
% input: filename = location of file to read in
% output: name = robot name
%         links = struct containing links that were read in
%         joints = struct containing joints that were read in
%
% written by: Cynthia Sung
% last modified: 07/20/2020
%

function [name, links, joints] = loadURDF(filename)

filepath = fileparts(filename);

ignore = {};
inignore = false; ignoreword = [];

name = '';
links = struct('name', {}, 'mesh', {}, 'origin', {}, 'childjoints', {});
joints = struct('name', {}, 'parent', {}, 'child', {}, 'origin', {}, 'axis', {});

fid = fopen(filename);

eof = false; tline = '';
while (~eof)
    % get line
    i = find(tline=='>',1,'first');
    if (isempty(i))
        [tline_new, eof] = getUntil(fid, '>');
        tline = [tline tline_new];
    end
    
    b = find(tline=='<',1,'first');
    if (isempty(b))
        b = 0;
    end
    
    i = find(tline=='>',1,'first');
    if (isempty(i))
        i = length(tline)+1;
    end
    
    % separate line to analyze
    toanalyze = tline(b+1:i-1);
    tline = tline(i+1:end);
    
    % are we in an ignore block?
    if (inignore)
        if strcmpi(toanalyze, ['/' ignoreword])
            inignore = false;
            ignoreword = [];
            fprintf('block done\n');
        end
    else
        % check all ignore words
        for i = 1:length(ignore)
            if strcmpi(ignore{i},toanalyze(1:min(length(ignore{i}),length(toanalyze))))
                inignore = true;
                ignoreword = ignore{i};
                fprintf(['Ignoring ''' ignoreword ''' block...']);
                break;
            end
        end
        
        % parse objects
        if (~inignore)
            % object type
            type = sscanf(toanalyze, '%s',1);
            
            switch(type)
                case 'robot'
                    idx = strfind(toanalyze, 'name');
                    colloc = find(toanalyze(idx:end)=='"',2,'first');
                    name = toanalyze(idx+colloc(1):idx+colloc(2)-2);
                case 'link'
                    idx = strfind(toanalyze, 'name');
                    colloc = find(toanalyze(idx:end)=='"',2,'first');
                    links(end+1).name = toanalyze(idx+colloc(1):idx+colloc(2)-2);
                    
                    [tline_new, eof] = getUntil(fid, '/link');
                    tline = [tline tline_new];
                    
                    idx = strfind(tline, 'rpy');
                    colloc = find(tline(idx:end)=='"',2,'first');
                    rpy = str2num(tline(idx+colloc(1):idx+colloc(2)-2));
                    
                    idx = strfind(tline, 'xyz');
                    colloc = find(tline(idx:end)=='"',2,'first');
                    links(end).origin = str2num(tline(idx+colloc(1):idx+colloc(2)-2));
                    
                    idx = strfind(tline, 'filename');
                    colloc = find(tline(idx:end)=='"',2,'first');
                    stlname = tline(idx+colloc(1):idx+colloc(2)-2);
                    
                    idx = strfind(tline, 'scale');
                    colloc = find(tline(idx:end)=='"',2,'first');
                    scale = str2num(tline(idx+colloc(1):idx+colloc(2)-2));
                    
                    [~,~,ext] = fileparts(stlname);
                    switch (ext)
                        case '.stl'
                            mesh = stlread(fullfile(filepath, stlname));
                        case '.obj'
                            obj = readObj(fullfile(filepath, stlname));
                            mesh = triangulation(obj.f.v, obj.v);
                    end
                    pts = mesh.Points;
                    pts = bsxfun(@times,pts,scale);
                    %pts = pts(:,[3 1 2]);
                    links(end).mesh = triangulation(mesh.ConnectivityList, pts);
                    
                    i = strfind(tline,'/link>');
                    tline = tline(i+6:end);
                case 'joint'
                    idx = strfind(toanalyze, 'name');
                    colloc = find(toanalyze(idx:end)=='"',2,'first');
                    joints(end+1).name = toanalyze(idx+colloc(1):idx+colloc(2)-2);
                    
                    [tline_new, eof] = getUntil(fid, '/joint');
                    tline = [tline tline_new];
                    
                    idx = strfind(tline, 'parent');
                    colloc = find(tline(idx:end)=='"',2,'first');
                    parent = tline(idx+colloc(1):idx+colloc(2)-2);
                    joints(end).parent = find(contains({links.name},parent), 1, 'first');
                    links(joints(end).parent).childjoints(end+1) = length(joints);
                    
                    idx = strfind(tline, 'child');
                    colloc = find(tline(idx:end)=='"',2,'first');
                    child = tline(idx+colloc(1):idx+colloc(2)-2);
                    joints(end).child = find(contains({links.name},child), 1, 'first');
                    
                    idx = strfind(tline, 'origin');
                    colloc = find(tline(idx:end)=='"',2,'first');
                    o = str2num(tline(idx+colloc(1):idx+colloc(2)-2));
                    %o = o(:,[3 1 2]);
                    joints(end).origin = o;
                    
                    idx = strfind(tline, 'axis');
                    colloc = find(tline(idx:end)=='"',2,'first');
                    a = str2num(tline(idx+colloc(1):idx+colloc(2)-2));
                    %a = a(:,[3 1 2]);
                    joints(end).axis = a;
                    
                    i = strfind(tline,'/joint>');
                    tline = tline(i+7:end);
                otherwise
                    disp(['Unknown object type ' type])
            end
        end
    end
end

fclose(fid);

return


function [tline, eof] = getUntil(fid, substr)

eof = false;
tline = fgetl(fid);
i = strfind(tline,substr);
while (isempty(i) && ~eof)
    toadd = fgetl(fid);
    if (ischar(toadd))
        tline = [tline toadd];
    else
        eof = true;
    end
    i = strfind(tline,substr);
end

return