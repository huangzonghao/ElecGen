function UI

%% INITIALIZATION
[flag, fignum] = figflag('ElecGen');
if flag
    close(fignum);
end

MainFigure = figure();
set(MainFigure, 'NumberTitle','off', 'Name', 'ElecGen', ...
    'MenuBar', 'none', 'Toolbar', 'none', ...
    'CloseRequestFcn', @checkquit, 'DeleteFcn', @quit);

leftwidth = .3;
LoadPanel = makePanel([0,.8,leftwidth,.2],'Load');
OutputPanel = makePanel([0,0,leftwidth,.8], 'Output');
ButtonsPanel = makePanel([leftwidth,.8,1-leftwidth,.2],'Functionalities');
RobotPanel = makePanel([leftwidth,.4,1-leftwidth,.4],'Robot');
EnvPanel = makePanel([leftwidth,0,1-leftwidth,.4],'Environment');

RobotAxes = makeAxes(RobotPanel, [0.05 0.05 0.7 0.9]);
EnvAxes = makeAxes(EnvPanel, [0.05 0.05 0.9 0.9]);

robotload_button = makeButton(LoadPanel, [.1, .6, .8, .3], 'Load Robot', @loadrobot);
envload_button = makeButton(LoadPanel, [.1, .1, .8, .3], 'Load Environment', @loadenvironment);
run_button = makeButton(EnvPanel, [.85, .05, .1, .15], 'RUN', @run);

envSTL = triangulation([1 2 3; 2 3 4], [0 0 0; 0 1 .01; 1 0 0; 1 1 0]);
robotName = '';
robotLinks = '';
robotJoints = '';

redrawRobot;
redrawEnv;

%% SIMULATION
    function run
    end

%% DRAWING
    function redrawRobot
        cla(RobotAxes);
        
        for i = 1:length(robotLinks)
            trisurf(robotLinks(i).mesh, 'Parent', RobotAxes, ...
                'FaceColor', [.8, .8, 1], 'FaceAlpha', .7, ...
                'EdgeColor', 'none',...
                'SpecularStrength', .5, 'AmbientStrength', .5);
            hold(RobotAxes, 'on');
            F = reduceEdges(robotLinks(i).mesh, pi/20)';
            pts = robotLinks(i).mesh.Points;
            plot3(reshape(pts(F,1),[],2)',...
                reshape(pts(F,2),[],2)',...
                reshape(pts(F,3),[],2)',...
                'k','LineWidth',1.5,'Parent',RobotAxes);
        end
        
        for i = 1:length(robotJoints)
            o = robotJoints(i).origin;
            a = robotJoints(i).axis;
            plot3([o(1) o(1)-.5*a(1)], [o(2) o(2)-.5*a(2)], [o(3) o(3)-.5*a(3)],...
                'r','LineWidth',3,'Parent',RobotAxes);
        end
        
        updateAxesProperties(RobotAxes);
    end

    function redrawEnv
        trisurf(envSTL, 'Parent', EnvAxes, ...
            'FaceColor', [.8, .8, .8], 'EdgeColor', 'none',...
            'SpecularStrength', .5, 'AmbientStrength', .5);
        updateAxesProperties(EnvAxes);
    end

%% LOAD FUNCTIONALITY
    function loadrobot(~,~)
        % load a URDF robot
        [filename, pathname] = uigetfile( ...
            {'*.urdf','URDF (*.urdf)';
            '*.*',  'All Files (*.*)'}, ...
            'Select a robot to load.',...
            '.\','MultiSelect', 'off');
        if isequal(filename,0)
            disp('No file selected')
        else
            disp(['Loading ' filename]);
            [robotName, robotLinks, robotJoints] = loadURDF(fullfile(pathname, filename));
            
            redrawRobot;
            
            warning('not implemented')
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
        panel = uipanel(MainFigure, 'Units', 'Normalized', 'Position', position,...
            'Title', title, 'FontSize', 12, 'BackgroundColor', [1 1 1]);
    end

    function ax = makeAxes(parent, position)
        ax = axes(parent, 'FontSize', 10, 'Units', 'Normalized', 'Position', position);
        updateAxesProperties(ax);
    end

    function updateAxesProperties(ax)
        axis(ax,'equal');
        axis(ax,'tight');
        axis(ax,'off');
        camup(ax,'manual')
        R = rotate3d(ax);
        R.Enable = 'on';
        R.ActionPostCallback = @(~,~)updateLight(ax);
        updateLight(ax);
    end

    function updateLight(ax)
        c = ax.Children;
        for i = 1:length(c)
            if isa(c(i), 'matlab.graphics.primitive.Light')
                delete(c(i));
            end
        end
        camlight(ax);
    end

    function button = makeButton(parent, position, buttonlbl, callback)
        button = uicontrol(parent, 'Style', 'pushbutton', 'String', buttonlbl, ...
            'units', 'normalized', 'position', position, 'fontsize', 10, ...
            'Callback', callback);
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