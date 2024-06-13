

%Perform P3P on a prerecorded video of a checkerboard

%Initialise trajectory variables
t_hist_cust = zeros(3,1); %trajectory history, in the form of translation vector
t_hist_builtin = zeros(3,1);

%Initialise camera variables
K = [1274.58238813250 0 627.693937913345; 0 1273.36371905619 405.441250468717; 0 0 1];

%Initialise targets (checkerboard)
checkerSize = [7, 10]; %number of squares
checkerSquareLength = 23; %mm

%Initialise world map
X_pnts_w = calcCheckerEdgeCoords_w(checkerSize, checkerSquareLength);


%Initialise trajectory plot
close all
trajPlotFig = figure(); 
trajPlotAx = axes('Parent', trajPlotFig)%, 'XLabel','x (mm)', 'YLabel', 'y (mm)', 'ZLabel', 'z (mm)');
xlim(trajPlotAx, [-500, 500]);
ylim(trajPlotAx,[-500, 500]);
zlim(trajPlotAx,[-500, 500]);
xlabel(trajPlotAx, 'x (mm)');
ylabel(trajPlotAx, 'y (mm)');
zlabel(trajPlotAx, 'z (mm)');
set(trajPlotAx, 'Ydir', 'reverse', 'Zdir', 'reverse');
view(trajPlotAx, 3);
grid(trajPlotAx, 'on');
hold (trajPlotAx, 'on');
trajPlotCust = plot3(trajPlotAx,[0],[0],[0], '-or');
trajPlotBuiltin = plot3(trajPlotAx,[0],[0],[0], '-og');
%Camera axes
camXPlotCust = plot3(trajPlotAx, [0 100], [0 0], [0 0], '-r');
camYPlotCust = plot3(trajPlotAx, [0 0], [0 100], [0 0], '-r');
camZPlotCust = plot3(trajPlotAx, [0 0], [0 0], [0 100], '-r');

camXPlotBuiltin = plot3(trajPlotAx, [0 100], [0 0], [0 0], '-g');
camYPlotBuiltin = plot3(trajPlotAx, [0 0], [0 100], [0 0], '-g');
camZPlotBuiltin = plot3(trajPlotAx, [0 0], [0 0], [0 100], '-g');
camXLabelBuiltin = text(trajPlotAx, 100,0, 0, 'x');
camYLabelBuiltin=text(trajPlotAx, 0, 100,0, 'y');
camZLabelBuiltin=text(trajPlotAx, 0, 0, 1, 'z');

%Insert checkerboard
scatter3(trajPlotAx,X_pnts_w(:,1), X_pnts_w(:,2),X_pnts_w(:,3), 4, "black", "filled");

%Locate video
files_ds = fileDatastore(fullfile(uigetdir), 'ReadFcn', @importdata, "FileExtensions",".jpg");
allFileNames = files_ds.Files;
numFiles = length(allFileNames);

%For first iteration
I_colour = imread(string(allFileNames(1)));
I_gray = rgb2gray(I_colour);
I_allFrames = I_gray;


%Loop to read all image files and put them into a multidim array
for n=2: numFiles

    I_colour = imread(string(allFileNames(n)));
    I_gray = rgb2gray(I_colour);
    I_allFrames = cat(3, I_allFrames, I_gray);
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%Do P3P!

%Initialise sim
mdl = "P3P_Grunert";
open_system(mdl);
simIn = Simulink.SimulationInput(mdl);



set_param(mdl,"FastRestart","on")




numFrames = numFiles;
for q=1:numFrames
    
    %Extract current frame
    I_current = I_allFrames(:,:,q);

    %Run simulation
    out = sim(simIn);
    
    %Load new augmented Rt matrix results
    R_new_cust = out.simout(1:3,1:3,1);
    t_new_cust = out.simout(1:3,4,1);
    R_new_builtin = out.simout(1:3,1:3,2);
    t_new_builtin = out.simout(1:3,4,2);

    %Add new results to trajectory history
    if q == 1
        t_hist_cust = t_new_cust;
        t_hist_builtin = t_new_builtin;
    else
        t_hist_cust = cat(2, t_hist_cust, t_new_cust);
        t_hist_builtin = cat(2, t_hist_builtin, t_new_builtin);
    end

    %Plot new trajectory history
    set(trajPlotCust, 'XData', t_hist_cust(1,:), 'YData',t_hist_cust(2,:), 'ZData', t_hist_cust(3,:));
    set(trajPlotBuiltin, 'XData', t_hist_builtin(1,:), 'YData',t_hist_builtin(2,:), 'ZData', t_hist_builtin(3,:));
    
    %Plot new camera axes
    %Custom algorithm
    cam_xAxis_cust = cat(2, t_new_cust, (t_new_cust + (R_new_cust * [100;0;0])));
    cam_yAxis_cust = cat(2, t_new_cust, (t_new_cust + (R_new_cust * [0;100;0])));
    cam_zAxis_cust = cat(2, t_new_cust, (t_new_cust + (R_new_cust * [0;0;100])));
    set(camXPlotCust, 'XData', cam_xAxis_cust(1,:), 'YData',cam_xAxis_cust(2,:), 'ZData', cam_xAxis_cust(3,:));
    set(camYPlotCust, 'XData', cam_yAxis_cust(1,:), 'YData',cam_yAxis_cust(2,:), 'ZData', cam_yAxis_cust(3,:));
    set(camZPlotCust, 'XData', cam_zAxis_cust(1,:), 'YData',cam_zAxis_cust(2,:), 'ZData', cam_zAxis_cust(3,:));
    
    %Builtin algorithm
    cam_xAxis_builtin = cat(2, t_new_builtin, (t_new_builtin + (R_new_builtin * [100;0;0])));
    cam_yAxis_builtin = cat(2, t_new_builtin, (t_new_builtin + (R_new_builtin * [0;100;0])));
    cam_zAxis_builtin = cat(2, t_new_builtin, (t_new_builtin + (R_new_builtin * [0;0;100])));
    set(camXPlotBuiltin, 'XData', cam_xAxis_builtin(1,:), 'YData',cam_xAxis_builtin(2,:), 'ZData', cam_xAxis_builtin(3,:));
    set(camYPlotBuiltin, 'XData', cam_yAxis_builtin(1,:), 'YData',cam_yAxis_builtin(2,:), 'ZData', cam_yAxis_builtin(3,:));
    set(camZPlotBuiltin, 'XData', cam_zAxis_builtin(1,:), 'YData',cam_zAxis_builtin(2,:), 'ZData', cam_zAxis_builtin(3,:));
    delete(camXLabelBuiltin);
    delete(camYLabelBuiltin);
    delete(camZLabelBuiltin);
    camXLabelBuiltin = text(trajPlotAx, cam_xAxis_builtin(1,2), cam_xAxis_builtin(2,2), cam_xAxis_builtin(3,2), 'x');
    camYLabelBuiltin =text(trajPlotAx, cam_yAxis_builtin(1,2), cam_yAxis_builtin(2,2), cam_yAxis_builtin(3,2), 'y');
    camZLabelBuiltin =text(trajPlotAx, cam_zAxis_builtin(1,2), cam_zAxis_builtin(2,2), cam_zAxis_builtin(3,2), 'z');

    drawnow


end



function X_pts_w = calcCheckerEdgeCoords_w(checkerSize, squareEdgeLength)
    
    boardWidth = checkerSize(2);
    boardHeight = checkerSize(1);
    X_pts_w = zeros((boardWidth-1)*(boardHeight-1), 3);

    %for each column
    for c=1 : boardWidth-1
        for r = 1: boardHeight-1
            x = (c-1)*squareEdgeLength;
            y = (r-1)*squareEdgeLength;
            z = 0;
            X_pts_w(  (((c-1)*(boardHeight-1)) + r), 1) = x;
            X_pts_w(  (((c-1)*(boardHeight-1)) + r), 2) = y;
            X_pts_w(  (((c-1)*(boardHeight-1)) + r), 3) = z;
        end
    end
end

