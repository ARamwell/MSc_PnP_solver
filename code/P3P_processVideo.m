

%Perform P3P on a prerecorded video of a checkerboard

%Locate video
[vidFile,vidLocation] = uigetfile('','Select a video to process');
dirOutput = dir(fullfile(vidLocation,vidFile));
fileNames = {dirOutput.name}'
numFrames = numel(fileNames)

I = imread(fullfile(vidLocation, vidFile));
vidImageSeq_BW = zeros([size(I) numFrames],class(I));
vidImageSeq_BW(:,:,1) = rgb2gray(I);

for p = 2:numFrames
    I = imread(fullfile(vidLocation, vidFile));
    vidImageSeq_BW(:,:,p) = rgb2gray(I);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%Maybe do some kind of processing here?
%Convert to grayscale
for p = 2:numFrames
    vidImageSeq_BW(:,:,p) = rgb2gray(vidImageSeq(:,:,p))
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%Do P3P!

%Initialise sim
mdl = "P3P_Grunert";
open_system(mdl);
simIn = Simulink.SimulationInput(mdl);
%p3pMask = Simulink.Mask.get("P3P_Grunert/Perform P3P and RANSAC on checkerboard");

%Set parameters
checkerSize = [7, 10]; %number of squares
checkerSquareLength = 23; %mm
K = [1274.58238813250 0 627.693937913345; 0 1273.36371905619 405.441250468717; 0 0 1];

%Pass to simulnk
%p3pMask.Parameters(2).set('Value', checkerSize);
%set_param(maskPath, 'checkerSize', checkerSize);
%set_param(p3pMask, 'K', K);
%set_param
%set_param(p3pMask, 'checkerSquareEdgeLength', checkerSquareLength);

%Create struct to pass parameters to simulation
%simData = createInputDataset(mdl);
%simData{2} = checkerSize;
%simData{3} = K;
%simData{4} = checkerSquareLength;
%simIn =simIn.setBlockParameter("P3P_Grunert/Perform P3P and RANSAC on checkerboard", 'checkerSize', checkerSize);
%simIn =simIn.setBlockParameter("P3P_Grunert/Perform P3P and RANSAC on checkerboard", 'intrinsicMat', K);
%simIn =simIn.setBlockParameter("P3P_Grunert/Perform P3P and RANSAC on checkerboard", 'checkerEdgeLength', checkerSquareLength);


simIn = simIn.setVariable('checkerSize', checkerSize, 'Workspace', mdl);
simIn = simIn.setVariable('checkerSquareEdgeLength', checkerSquareLength, 'Workspace', mdl);
simIn = simIn.setVariable('K', K, 'Workspace', mdl);



for q=1:numFrames

    %simData{1} = vidImageSeq_BW(:,:,q);
    I_current = vidImageSeq_BW(:,:,q);
    simIn = simIn.setVariable('intensityMatrix', I_current, 'Workspace', mdl);
    %Load data into sim
    %simIn = setExternalInput(simIn,inDS);
    %set_param(p3pMask, 'intensityMatrix', vidImageSeq_BW(:,:,q));

    out = sim(simIn);
end




   

