function UI

%% INITIALIZATION
[flag, fignum] = figflag('ElecGen');
if flag
    close(fignum);
end

urdf_filename = '';

% IO file locations
outputfile = fullfile('..', 'output.txt');
output_urdffile = fullfile('..', 'data', 'robots', 'robot.urdf.tmp');

% colors
bckclr = [1 1 1];
jointColor_deselect = [0, 0, 1]; jointColor_select = [1, 0, 0];
faceColor_deselect = [.7, .7, 1]; faceColor_select = [1, .5, .5];

% figure
MainFigure = figure('NumberTitle','off', 'Name', 'ElecGen', ...
    'MenuBar', 'none', 'Toolbar', 'none', ...
    'WindowButtonMotionFcn', @figureWindowButtonMoveCallback,...
    'WindowButtonUpFcn', @figureWindowButtonUpCallback,...
    'DeleteFcn', @quit);

% panels
leftwidth = .3;
LoadPanel = makePanel([0,.8,leftwidth,.2],'Load');
OutputPanel = makePanel([0,0,leftwidth,.8], 'Output');
outputText = uicontrol(OutputPanel, 'style', 'text', ...
    'units', 'normalized', 'position', [.01 .01 .99 .99], ...
    'horizontalalignment','left','backgroundcolor', bckclr);
RobotPanel = makePanel([leftwidth,.4,1-leftwidth,.6],'Robot');
EnvPanel = makePanel([leftwidth,0,1-leftwidth,.4],'Environment');

% buttons
robotload_button = makeButton(LoadPanel, [.1, .6, .8, .3], 'Load Robot', @loadrobot);
envload_button = makeButton(LoadPanel, [.1, .1, .8, .3], 'Load Environment', @loadenvironment);
clear_output_button = makeButton(OutputPanel, [.5, .01, .45, .08], 'Clear', @clearOutput);
run_button = makeButton(EnvPanel, [.85, .05, .1, .15], 'RUN', @run);
clear_traj_button = makeButton(EnvPanel, [.05, .05, .1, .15], 'Clear', @clearTrajectory);

ButtonFunctions = {'DC motor', 'Servomotor', 'Force<br>sensing', 'Velocity<br>control', 'Remote<br>control', 'Line<br>tracking'};
FunctionType = [1, 1, 0, 0, 0, 0];  % 1 = actuator, 0 = sensor
buttons = populateButtonsPanel(ButtonFunctions);

% axes
mousedata = initializeMouseData;
RobotAxes = makeAxes(RobotPanel, [0.05 0.05 0.7 0.9], 'robot');
EnvAxes = makeAxes(EnvPanel, [0.05 0.05 0.9 0.9], 'environment');

% data
envFile = '';
envMesh = triangulation([1 2 3; 2 3 4], [0 0 0; 0 1 .01; 1 0 0; 1 1 0]);
trajectory = zeros(0,3);
traj_handle = []; traj_proj_handle = [];
dist_thresh = 0.05; traj_zoffset = 0.01;

robotName = '';
robotLinks = '';
robotJoints = '';

selected = struct('type', 'none', 'id', [], 'handle', []);

% update drawings
redrawRobot;
redrawEnv;

%% SIMULATION
    function run(~,~)
        addpath('../build');
        mexRun(0, 0, 10, 0, urdf_filename, envFile);
        writeURDF(output_urdffile, robotName, robotLinks, robotJoints, ButtonFunctions, FunctionType);

        displayOutput;
    end

%% OUTPUT
    function displayOutput(~,~)
        fid = fopen(outputfile);
        % read in the whole file
        tline = '';
        while ~feof(fid)
            tline = [tline fgets(fid)];
        end
        
        outputText.String = tline;
    end

    function clearOutput(~,~)
        outputText.String = '';
    end

%% FUNCTIONALITY BUTTON UPDATES
    function switchButtonState(i)
        % toggle states of functionality buttons
        switch(selected.type)
            case 'joint'
                robotJoints(selected.id).buttonState(i) = ...
                    ~robotJoints(selected.id).buttonState(i);
            case 'link'
                robotLinks(selected.id(1)).buttonState(selected.id(2),i) = ...
                    ~robotLinks(selected.id(1)).buttonState(selected.id(2),i);
        end
    end

    function deactivateButtons
        for ibutton = 1:length(buttons)
            buttons(ibutton).Enable = 'off';
            buttons(ibutton).Value = 0;
        end
    end

    function activateButtons
        switch(selected.type)
            case 'joint'
                values = robotJoints(selected.id).buttonState;
            case 'link'
                values = robotLinks(selected.id(1)).buttonState(selected.id(2),:);
        end
        
        for ibutton = 1:length(buttons)
            if strcmp(selected.type, 'joint') || ~FunctionType(ibutton)
                buttons(ibutton).Enable = 'on';
            end
            buttons(ibutton).Value = values(ibutton);
        end
    end

%% ROBOT PART SELECTION
    function patchClick(src,event,ilink,igroup)
        if strcmp(MainFigure.SelectionType, 'alt')
            axesButtonDownCallback(src,event);
            return;
        end
        
        drawDeselect;
        
        if strcmp(selected.type, 'link') && all(selected.id == [ilink igroup])
            selected.type = 'none';
        else
            selected.type = 'link';
            selected.id = [ilink igroup];
            selected.handle = src;
            drawSelect;
        end
    end

    function jointClick(src,event,ijoint)
        if strcmp(MainFigure.SelectionType, 'alt')
            axesButtonDownCallback(src,event);
            return;
        end
        
        drawDeselect;
        
        if strcmp(selected.type, 'joint') && selected.id == ijoint
            selected.type = 'none';
        else
            selected.type = 'joint';
            selected.id = ijoint;
            selected.handle = src;
            drawSelect;
        end
    end

%% ENVIRONMENT TRAJECTORY UPDATES
    function envClick(src,event)
        if strcmp(MainFigure.SelectionType, 'alt')
            axesButtonDownCallback(src,event);
            return;
        end
        
        pt = findIntersection(EnvAxes.CurrentPoint, envMesh);
        trajectory(end+1,:) = pt;
        redrawTrajectory;
    end

    function clearTrajectory(~,~)
        trajectory = zeros(0,3);
        delete(traj_handle);
        delete(traj_proj_handle);
        traj_handle = [];
        traj_proj_handle = [];
    end

    function clickTrajectoryLineCallback(~,evt)
        % modify the trajectory
        pt = evt.IntersectionPoint;
        pt(3) = pt(3) - traj_zoffset;
        
        % find segment based on xy projection
        dir = trajectory(2:end,1:2)-trajectory(1:end-1,1:2);
        dist = sqrt(sum(dir.^2,2));
        diff = pt(1:2) - trajectory(1:end,1:2);
        proj = dot(diff(1:end-1,:), dir, 2)./dist.^2;
        proj(proj < 0) = 0; proj(proj > 1) = 1;
        perp = diff(1:end-1,:) - bsxfun(@times,proj,dir);
        [~,idx] = min(sum(perp.^2,2));
        
        dist = sum(diff(idx:idx+1,:).^2,2);
        
        if dist(1) > dist_thresh^2 && dist(2) > dist_thresh^2 && evt.Button == 1
            % where on the line is the point located?
            switch (idx)
                case 1
                    trajectory = [trajectory(1,:); pt; trajectory(2:end,:)];
                case size(trajectory,1)
                    trajectory = [trajectory(1:end-1,:); pt; trajectory(end,:)];
                otherwise
                    trajectory = [trajectory(1:idx,:); pt; trajectory(idx+1:end,:)];
            end
        end
        
        redrawTrajectory;
    end

    function clickTrajectoryPointCallback(~,evt)
        % modify the trajectory
        pt = evt.IntersectionPoint;
        pt(3) = pt(3) - traj_zoffset;
        
        dist = sum((pt - trajectory(1:end,:)).^2,2);
        [~,idx] = min(dist);
        
        switch (evt.Button)
            case 1 % normal
                mousedata.pressed = true;
                mousedata.mouse_button = 'trajectory';
                mousedata.traj_idx = idx;
            case 3 % right click
                trajectory(idx,:) = [];
        end
        
        redrawTrajectory;
    end

%% AXES ROTATION MOUSE CLICK CALLBACKS
    function axesButtonDownCallback(~, ~)
        mousedata.pressed = true;
        mousedata.mouse_button = get(MainFigure, 'SelectionType');
        mousedata.position = get(0, 'PointerLocation');
        
        switch (mousedata.mouse_button)
            case 'alt'    % right click
                mousedata.mouse_button = 'rotate';
                MainFigure.Pointer = 'circle';
            case 'normal' % left click
                MainFigure.Pointer = 'arrow';
        end
    end

    function figureWindowButtonMoveCallback(~, ~)
        mousedata.position_last = mousedata.position;
        mousedata.position = get(0, 'PointerLocation');
        
        if mousedata.pressed
            dp = mousedata.position_last - mousedata.position;
            
            switch mousedata.mouse_button
                case 'rotate'
                    UpVector = get(gca, 'CameraUpVector');
                    XYZ = get(gca,'CameraPosition');
                    Camtar = get(gca,'CameraTarget');
                    Forward = (Camtar-XYZ) / norm(Camtar-XYZ);
                    ViewAngle = get(gca,'CameraViewAngle');
                    
                    Mview = [UpVector; Forward; cross(UpVector, Forward)];
                    R = rotationMatrix([dp(1) 0 dp(2)]);
                    Mview = R'*Mview;
                    
                    UpVector = Mview(1,1:3);
                    XYZ = Camtar - norm(Camtar-XYZ)*Mview(2,1:3);
                    
                    set(gca,'CameraUpVector', UpVector);
                    set(gca,'CameraPosition', XYZ);
                    set(gca,'CameraTarget', Camtar);
                    set(gca,'CameraViewAngle', ViewAngle);
                case 'normal'
            end
        end
    end

    function figureWindowButtonUpCallback(~, ~)
        if (mousedata.pressed && strcmp(mousedata.mouse_button, 'trajectory'))
            pt = findIntersection(EnvAxes.CurrentPoint, envMesh);
            trajectory(mousedata.traj_idx,:) = pt;
            redrawTrajectory;
        end
        
        mousedata.mouse_pressed = false;
        MainFigure.Pointer = 'arrow';
        mousedata.mouse_button = '';
    end

    function R = rotationMatrix(r)
        % Determine the rotation matrix (View matrix) for rotation angles xyz ...
        Rx = [1 0 0; 0 cosd(r(1)) -sind(r(1)); 0 sind(r(1)) cosd(r(1))];
        Ry = [cosd(r(2)) 0 sind(r(2)); 0 1 0; -sind(r(2)) 0 cosd(r(2))];
        Rz = [cosd(r(3)) -sind(r(3)) 0; sind(r(3)) cosd(r(3)) 0; 0 0 1];
        R = Rx*Ry*Rz;
    end

    function mousedata = initializeMouseData
        mousedata.position = [0 0];
        mousedata.position_last = [0 0];
        mousedata.pressed = false;
        mousedata.mouse_button = '';
        mousedata.traj_idx = 0;
    end

%% DRAWING ROBOT
    function redrawRobot
        % update robot drawing (called upon loading a new robot)
        
        cla(RobotAxes);
        
        if ~isempty(robotLinks)
            link_queue = [1; 0]; % assume link 1 is the root
            while ~isempty(link_queue)
                i = link_queue(:,1); link_queue(:,1) = [];
                
                % transform mesh according to joint
                mesh = robotLinks(i(1)).mesh;
                pts = mesh.Points;
                if i(2)>0
                    % FIXME: won't work with tree depth > 1
                    o = robotJoints(i(2)).origin;
                    rpy = robotJoints(i(2)).rpy;
                    rotm = eul2rotm(rpy, 'XYZ');
                    
                    % draw link
                    pts = (rotm*pts')';
                    pts = bsxfun(@plus, pts, o+robotLinks(i(1)).origin);
                    mesh = triangulation(mesh.ConnectivityList, pts);
                    
                    % draw joint
                    a = robotJoints(i(2)).axis;
                    a = (rotm*a')';
                    plot3([o(1)+.25*a(1) o(1)-.25*a(1)], [o(2)+.25*a(2) o(2)-.25*a(2)], [o(3)+.25*a(3) o(3)-.25*a(3)],...
                        '-', 'Color', jointColor_deselect,'LineWidth',3,'Parent',RobotAxes,...
                        'ButtonDownFcn', @(s,e)jointClick(s,e,i(2)));
                    
                    robotJoints(i(2)).buttonState = false(1,length(ButtonFunctions));
                end
                
                % draw link
                [edges, grp_assign] = reduceEdges(mesh, pi/8);
                for igroup = unique(grp_assign(:)')
                    patch('Faces', mesh.ConnectivityList(grp_assign==igroup,:), 'Vertices', mesh.Points, ...
                        'Parent', RobotAxes, ...
                        'FaceColor', faceColor_deselect, 'FaceAlpha', .7, ...
                        'EdgeColor', 'none',...
                        'SpecularStrength', .5, 'AmbientStrength', .5, ...
                        'ButtonDownFcn', @(s,e)patchClick(s,e,i(1),igroup));
                end
                robotLinks(i(1)).face_groups = grp_assign;
                robotLinks(i(1)).buttonState = false(length(unique(grp_assign)),length(ButtonFunctions));
                
                plot3(reshape(pts(edges,1),[],2)',...
                    reshape(pts(edges,2),[],2)',...
                    reshape(pts(edges,3),[],2)',...
                    '-','Color',[0, 0, .5],'LineWidth',1.5,'Parent',RobotAxes);
                
                % get children
                childjoints = robotLinks(i(1)).childjoints;
                children = [robotJoints(childjoints).child];
                link_queue(:, end+1:end+length(childjoints)) = [children; childjoints];
            end
        end
        
        updateCamera(RobotAxes)
        updateLight(RobotAxes);
    end

    function drawSelect
        % highlight selected object
        
        if strcmp(selected.type,'joint')
            % update plot
            selected.handle.Color = jointColor_select;
        elseif strcmp(selected.type, 'link')
            selected.handle.FaceColor = faceColor_select;
        end
        
        activateButtons
    end

    function drawDeselect
        % unhighlight selected object
        
        if strcmp(selected.type,'joint')
            % update plot
            selected.handle.Color = jointColor_deselect;
        elseif strcmp(selected.type, 'link')
            selected.handle.FaceColor = faceColor_deselect;
        end
        
        deactivateButtons;
    end

%% DRAWING ENVIRONMENT
    function redrawEnv
        % redraw environment
        cla(EnvAxes)
        trisurf(envMesh, 'Parent', EnvAxes, ...
            'FaceColor', [.8, .8, .8], 'EdgeColor', 'none',...
            'SpecularStrength', .5, 'AmbientStrength', .5, 'ButtonDownFcn', @envClick);
        updateCamera(EnvAxes)
        updateLight(EnvAxes);
    end

    function redrawTrajectory
        if size(trajectory,1) > 1
            traj_proj = projectOnHeightMap(trajectory, envMesh);
        else
            traj_proj = zeros(0,3);
        end
        if isempty(traj_proj_handle)
            traj_proj_handle = plot3(traj_proj(:,1),traj_proj(:,2),traj_proj(:,3)+traj_zoffset,...
                '-r', 'linewidth', 2, ...
                'buttondownfcn', @clickTrajectoryLineCallback);
        else
            set(traj_proj_handle , 'xdata', traj_proj(:,1), 'ydata', traj_proj(:,2), 'zdata', traj_proj(:,3)+traj_zoffset);
        end
        
        if isempty(traj_handle)
            traj_handle = plot3(trajectory(:,1), trajectory(:,2), trajectory(:,3)+traj_zoffset, ...
                '.r','markersize', 20, 'linewidth', 2, ...
                'buttondownfcn', @clickTrajectoryPointCallback);
        else
            set(traj_handle, 'xdata', trajectory(:,1), 'ydata', trajectory(:,2), 'zdata', trajectory(:,3)+traj_zoffset);
        end
    end

%% GENERAL DRAWING
    function updateCamera(ax)
        xlim = get(ax, 'xlim');
        ylim = get(ax, 'ylim');
        zlim = get(ax, 'zlim');
        view(ax,3);
        ax.CameraTarget = [(xlim(1)+xlim(2))/2 (ylim(1)+ylim(2))/2 (zlim(1)+zlim(2))/2];
    end

    function updateLight(ax)
        % update light location so that it is always behind the camera
        if ~exist('ax','var')
            ax = gca;
        end
        c = ax.Children;
        for i = 1:length(c)
            if isa(c(i), 'matlab.graphics.primitive.Light')
                delete(c(i));
            end
        end
        camlight(ax);
    end

%% LOAD FUNCTIONALITY
    function loadrobot(~,~)
        % load a URDF robot
        
        [filename, pathname] = uigetfile( ...
            {'*.urdf','URDF (*.urdf)';
            '*.*',  'All Files (*.*)'}, ...
            'Select a robot to load.',...
            fullfile('..', 'data', 'robots'),'MultiSelect', 'off');
        if isequal(filename,0)
            disp('No file selected')
        else
            disp(['Loading ' filename]);
            urdf_filename = fullfile(pathname, filename);
            [robotName, robotLinks, robotJoints] = loadURDF(urdf_filename);
            
            redrawRobot;
        end
        
        selected.type = 'none';
        clearOutput
    end

    function loadenvironment(~,~)
        % load a STL environment
        
        [filename, pathname] = uigetfile( ...
            {'*.bmp', 'Bitmap (*.bmp)';
            '*.stl','STL (*.stl)';
            '*.obj', 'OBJ (*.obj)';
            '*.*',  'All Files (*.*)'}, ...
            'Select an environment to load.',...
            fullfile('..', 'data', 'maps'),'MultiSelect', 'off');
        
        if isequal(filename,0)
            disp('No file selected')
        else
            [~,~,ext] = fileparts(filename);
            envFile = fullfile(pathname, filename);
            switch (ext)
                case '.stl'
                    envMesh = stlread(envFile);
                case '.obj'
                    obj = readObj(envFile);
                    envMesh = triangulation(obj.f.v, obj.v);
                case '.bmp'
                    heightmap = imread(envFile);
                    if (size(heightmap,3) > 1)
                        heightmap = rgb2gray(heightmap);
                    end
                    
                    [X, Y] = meshgrid(linspace(0,1,size(heightmap,1)), linspace(0,1,size(heightmap,2)));
                    faces = delaunay(X, Y);
                    envMesh = triangulation(faces, X(:), Y(:), double(heightmap(:))/255*0.2);
            end
            
            V = envMesh.Points;
            dist = max(V,[],1)-min(V,[],1);
            dist_thresh = min(dist) / 20;
            
            disp(['Loaded environment: ' filename]);
            
            redrawEnv;
        end
        
        clearTrajectory
        clearOutput
    end

%% FIGURE SETUP
    function panel = makePanel(position, title)
        % make a new panel in the figure
        panel = uipanel(MainFigure, 'Units', 'Normalized', 'Position', position,...
            'Title', title, 'FontSize', 12, 'BackgroundColor', bckclr);
    end

    function ax = makeAxes(parent, position, name)
        % add axes to a panel
        ax = axes(parent, 'Tag',name, 'FontSize', 10, ...
            'Units', 'Normalized', 'Position', position,...
            'XColor','none', 'YColor','none', 'ZColor','none', ...
            'ButtonDownFcn', @axesButtonDownCallback);
        
        % update axes: square, 3d rotate on, camera light
        axis(ax,'equal');
        axis(ax,'tight');
        view(3);
        set(ax,'NextPlot','add');
        updateLight(ax);
    end

    function button = makeButton(parent, position, buttonlbl, callback)
        % add a button to the panel
        button = uicontrol(parent, 'Style', 'pushbutton', 'String', buttonlbl, ...
            'units', 'normalized', 'position', position, 'fontsize', 10, ...
            'Callback', callback);
    end

    function buttons = populateButtonsPanel(buttonnames)
        % add all buttons to the buttons panel
        % input is a cell of button names
        Nbuttons = length(buttonnames);
        margin_left = 0.75;
        margin_right = 0.02;
        ymargin = 0.05;
        height = (1-ymargin)/(Nbuttons+ymargin);
        
        for ibutton = 1:Nbuttons
            buttonlbl = ['<html><center>' buttonnames{ibutton} '</center></html>'];
            
            buttons(ibutton) = uicontrol(RobotPanel, 'Style', 'togglebutton', 'String', buttonlbl, ...
                'units', 'normalized', 'position', [margin_left, 1-(ibutton*(1+ymargin))*height, 1-margin_left-margin_right, height], ...
                'enable','off','fontsize', 10, 'callback', @(~,~)switchButtonState(ibutton));
        end
    end

%% QUIT FUNCTIONALITY
    function quit(~,~)
        disp('BYE!')
    end

end
