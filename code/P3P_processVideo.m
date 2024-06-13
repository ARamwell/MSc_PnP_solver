

%Perform P3P on a prerecorded video of a checkerboard

%Initialise variables
t_hist_cust = zeros(3,1); %trajectory history, in the form of translation vector
t_hist_builtin = zeros(3,1);

%Initialise trajectory plot
%figure;
trajPlotFig = figure(); 
trajPlotAx = axes('Parent', trajPlotFig);%'XLim', [-5000, 5000], 'YLim', [-5000, 5000], 'ZLim', [0, 5000]);%, 'XLabel', 'x (mm)', 'YLabel', 'y (mm)','ZLabel', 'z (mm)');

%trajPlot = plot3(t_hist_cust(1,:), t_hist_cust(2,:), t_hist_cust(3,:));

%plot3(trajPlotAx, [],[],[]);
xlim(trajPlotAx, [-5000, 5000]);
ylim(trajPlotAx,[-5000, 5000]);
zlim(trajPlotAx,[0, 5000]);
view(trajPlotAx, 3);
%hold on
%xlabel('x (mm)');
%ylabel('y (mm)');
%zlabel('z (mm)');
hold on
trajPlot = plot3(trajPlotAx,[1000],[1000],[1000], '-or');
%linkdata(trajPlotFig, 'on');
x =[1000 2000];
y = [1000 2000];
z=[1000 1000];
set(trajPlot, 'XData', x, 'YData', y, 'ZData', z);
drawnow
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

%Set parameters
checkerSize = [7, 10]; %number of squares
checkerSquareLength = 23; %mm
K = [1274.58238813250 0 627.693937913345; 0 1273.36371905619 405.441250468717; 0 0 1];

set_param(mdl,"FastRestart","on")




numFrames = numFiles;
for q=1:numFrames
    
    %Extract current frame
    I_current = I_allFrames(:,:,q);

    %Run simulation
    out = sim(simIn);
    
    %Load new augmented Rt matrix results
    Rt_new_cust = out.simout(:,:,1);
    Rt_new_builtin = out.simout(:,:,2);

    %Add new results to trajectory history
    if q == 1
        t_hist_cust = Rt_new_cust(1:3,4);
        t_hist_builtin = Rt_new_cust(1:3,4);
    else
        t_hist_cust = cat(2, t_hist_cust, Rt_new_cust(1:3,4));
        t_hist_builtin = cat(2, t_hist_builtin, Rt_new_builtin(1:3,4));
    end


    %Plot result
    drawnow



end

figure
trajScatter = scatter3(t_hist_cust(1,:), t_hist_cust(2,:), t_hist_cust(3,:))



   

