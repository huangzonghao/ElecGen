function UI

%% INITIALIZATION
[flag, fignum] = figflag('ElecGen');
if flag
    close(fignum);
end

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
ButtonsPanel = makePanel([leftwidth,.8,1-leftwidth,.2],'Functionalities');
RobotPanel = makePanel([leftwidth,.4,1-leftwidth,.4],'Robot');
EnvPanel = makePanel([leftwidth,0,1-leftwidth,.4],'Environment');

% buttons
robotload_button = makeButton(LoadPanel, [.1, .6, .8, .3], 'Load Robot', @loadrobot);
envload_button = makeButton(LoadPanel, [.1, .1, .8, .3], 'Load Environment', @loadenvironment);
run_button = makeButton(EnvPanel, [.85, .05, .1, .15], 'RUN', @run);

ButtonFunctions = {'Force<br>sensing', 'Velocity<br>control', 'Remote<br>control', 'Line<br>tracking'};
populateButtonsPanel(ButtonFunctions);
buttonState = false(1,length(ButtonFunctions));

% axes
mousedata = initializeMouseData;
RobotAxes = makeAxes(RobotPanel, [0.05 0.05 0.7 0.9], 'robot');
EnvAxes = makeAxes(EnvPanel, [0.05 0.05 0.9 0.9], 'environment');

% data
envSTL = triangulation([1 2 3; 2 3 4], [0 0 0; 0 1 .01; 1 0 0; 1 1 0]);
robotName = '';
robotLinks = '';
robotJoints = '';

% update drawings
redrawRobot;
redrawEnv;

%% SIMULATION
    function run
    end

%% STATE UPDATE CALLBACKS
    function switchButtonState(i)
        % toggle states of functionality buttons
        buttonState(i) = ~buttonState(i);
    end

    function axesButtonDownCallback(src, event)
        mousedata.pressed = true;
        mousedata.mouse_button = get(MainFigure, 'SelectionType');
        
        switch (mousedata.mouse_button)
            case 'alt'    % right click
                mousedata.mouse_button = 'rotate';
                set(MainFigure, 'Pointer','circle')
            case 'normal' % left click
                set(MainFigure, 'Pointer','arrow')
        end
    end

    function figureWindowButtonMoveCallback(src, event)
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

    function figureWindowButtonUpCallback(src, event)
        mousedata.mouse_pressed = false;
        set(MainFigure, 'Pointer','arrow')
        mousedata.mouse_button = '';
    end

    function mousedata = initializeMouseData
        mousedata.position = [0 0];
        mousedata.position_last = [0 0];
        mousedata.pressed = false;
        mousedata.mouse_button = '';
    end

%% CALLBACK HELPERS
    function R = rotationMatrix(r)
        % Determine the rotation matrix (View matrix) for rotation angles xyz ...
        Rx = [1 0 0; 0 cosd(r(1)) -sind(r(1)); 0 sind(r(1)) cosd(r(1))];
        Ry = [cosd(r(2)) 0 sind(r(2)); 0 1 0; -sind(r(2)) 0 cosd(r(2))];
        Rz = [cosd(r(3)) -sind(r(3)) 0; sind(r(3)) cosd(r(3)) 0; 0 0 1];
        R = Rx*Ry*Rz;
    end

%% DRAWING
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
                    pts = bsxfun(@plus, pts, robotJoints(i(2)).origin+robotLinks(i(1)).origin);
                    mesh = triangulation(mesh.ConnectivityList, pts);
                    
                    % draw joint
                    o = robotJoints(i(2)).origin;
                    a = robotJoints(i(2)).axis;
                    plot3([o(1)+.25*a(1) o(1)-.25*a(1)], [o(2)+.25*a(2) o(2)-.25*a(2)], [o(3)+.25*a(3) o(3)-.25*a(3)],...
                        'r','LineWidth',3,'Parent',RobotAxes);
                end
                
                % draw link
                trisurf(mesh, 'Parent', RobotAxes, ...
                    'FaceColor', [.8, .8, 1], 'FaceAlpha', .7, ...
                    'EdgeColor', 'none',...
                    'SpecularStrength', .5, 'AmbientStrength', .5);
                
                F = reduceEdges(mesh, pi/20)';
                plot3(reshape(pts(F,1),[],2)',...
                    reshape(pts(F,2),[],2)',...
                    reshape(pts(F,3),[],2)',...
                    'k','LineWidth',1.5,'Parent',RobotAxes);
                
                % get children
                childjoints = robotLinks(i(1)).childjoints;
                children = [robotJoints(childjoints).child];
                link_queue(:, end+1:end+length(childjoints)) = [children; childjoints];
            end
        end
        
        updateCamera(RobotAxes)
        updateLight(RobotAxes);
    end

    function redrawEnv
        % redraw environment
        cla(EnvAxes)
        trisurf(envSTL, 'Parent', EnvAxes, ...
            'FaceColor', [.8, .8, .8], 'EdgeColor', 'none',...
            'SpecularStrength', .5, 'AmbientStrength', .5, 'HitTest', 'off');
        updateLight(EnvAxes);
    end

    function updateCamera(ax)
        xlim = get(ax, 'xlim');
        ylim = get(ax, 'ylim');
        zlim = get(ax, 'zlim');
        view(ax,3);
        set(ax,'CameraTarget',[(xlim(1)+xlim(2))/2 (ylim(1)+ylim(2))/2 (zlim(1)+zlim(2))/2]);
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
            '..\robots\','MultiSelect', 'off');
        if isequal(filename,0)
            disp('No file selected')
        else
            disp(['Loading ' filename]);
            [robotName, robotLinks, robotJoints] = loadURDF(fullfile(pathname, filename));
            
            redrawRobot;
        end
    end

    function loadenvironment(~,~)
        % load a STL environment
        
        [filename, pathname] = uigetfile( ...
            {'*.stl','STL (*.stl)';
            '*.*',  'All Files (*.*)'}, ...
            'Select an environment to load.',...
            '.\','MultiSelect', 'off');
        
        if isequal(filename,0)
            disp('No file selected')
        else
            envSTL = stlread(fullfile(pathname, filename));
            
            disp(['Loaded environment: ' filename]);
            
            redrawEnv;
        end
    end

%% FIGURE SETUP
    function panel = makePanel(position, title)
        % make a new panel in the figure
        panel = uipanel(MainFigure, 'Units', 'Normalized', 'Position', position,...
            'Title', title, 'FontSize', 12, 'BackgroundColor', [1 1 1]);
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
        xmargin = 0.05;
        ymargin = 0.05;
        width = (1-xmargin)/(Nbuttons+xmargin);
        height = 1-2*ymargin;
        
        for ibutton = 1:Nbuttons
            buttonlbl = ['<html><center>' buttonnames{ibutton} '</center></html>'];
            
            buttons(ibutton) = uicontrol(ButtonsPanel, 'Style', 'togglebutton', 'String', buttonlbl, ...
                'units', 'normalized', 'position', [(xmargin+(ibutton-1)*(1+xmargin))*width, ymargin, width, height], ...
                'fontsize', 10, 'callback', @(~,~)switchButtonState(ibutton));
        end
    end

%% QUIT FUNCTIONALITY
    function checkquit(~,~)
%         selection = questdlg('Are you sure you want to exit?',...
%             'Exit?',...
%             'Yes','No','Yes');
%         switch selection
%             case 'Yes'
%                 delete(gcf)
%             case 'No'
%                 return
%         end
        delete(MainFigure);
    end

    function quit(~,~)
        disp('BYE!')
    end

end