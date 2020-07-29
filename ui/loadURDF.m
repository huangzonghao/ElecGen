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

ignore = {'motor', 'extra_mass'};
inignore = false; ignoreword = [];

name = '';
links = struct('name', {}, 'mesh', {}, 'origin', {}, 'childjoints', {}, 'text', {});
joints = struct('name', {}, 'parent', {}, 'child', {}, 'origin', {}, 'axis', {}, 'text', {});

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
                    name = getvalue(toanalyze, 'name');
                case 'link'
                    links(end+1).name = getvalue(toanalyze, 'name');
                    
                    [tline_new, eof] = getUntil(fid, '/link');
                    tline = ['<' toanalyze tline tline_new];
                    
                    rpy = str2num(getvalue(tline, 'rpy'));
                    links(end).origin = str2num(getvalue(tline, 'xyz'));
                    scale = str2num(getvalue(tline, 'scale'));
                    
                    stlname = getvalue(tline, 'filename');
                    
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
                    links(end).text = tline(1:i+5);
                    tline = tline(i+6:end);
                case 'joint'
                    joints(end+1).name = getvalue(toanalyze, 'name');
                    
                    [tline_new, eof] = getUntil(fid, '/joint');
                    tline = ['<' toanalyze tline tline_new];
                    
                    parent = getvalue(tline, 'parent');
                    joints(end).parent = find(contains({links.name},parent), 1, 'first');
                    links(joints(end).parent).childjoints(end+1) = length(joints);
                    
                    child = getvalue(tline, 'child');
                    joints(end).child = find(contains({links.name},child), 1, 'first');
                    
                    o = str2num(getvalue(tline, 'origin'));
                    %o = o(:,[3 1 2]);
                    joints(end).origin = o;
                    
                    a = str2num(getvalue(tline, 'axis'));
                    %a = a(:,[3 1 2]);
                    joints(end).axis = a;
                    
                    i = strfind(tline,'/joint>');
                    joints(end).text = tline(1:i+6);
                    tline = tline(i+7:end);
                case {'?xml','/robot',0}
                otherwise
                    disp(['Unknown object type ' type])
            end
        end
    end
end

fclose(fid);

return


function txt = getvalue(toanalyze, key)

idx = strfind(toanalyze, key);
colloc = find(toanalyze(idx:end)=='"',2,'first');
txt = toanalyze(idx+colloc(1):idx+colloc(2)-2);

return


function [tline, eof] = getUntil(fid, substr)

eof = false;
tline = fgets(fid);
i = strfind(tline,substr);
while (isempty(i) && ~eof)
    toadd = fgets(fid);
    if (ischar(toadd))
        tline = [tline toadd];
    else
        eof = true;
    end
    i = strfind(tline,substr);
end

return