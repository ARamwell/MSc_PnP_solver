classdef imgFuncs
    methods (Static)

        %------------------------------------------------------------%
        function [rtHist, timeHist, poseHist] = importSimLog(fullFile, R_sim2W)
        %Basic function to import all logged trajectory data (as .m file). 
        %Does not consider time alignment with other data (i.e., imports 
        %all logged points, does not skip any)
            
            simData = load(fullFile);
            numLogPoints = size((simData.out.camPose.signals.values), 3);

            %Initialise output variables
            rtHist = zeros(3,4,numLogPoints-1);%-1 to skip t0
            timeHist = zeros(1, (numLogPoints-1));
            poseHist =zeros(6, (numLogPoints-1));


            %For each logged time - skip t0, where there is no data
            for i=2:numLogPoints
                %Import time history
                time = simData.out.camPose.time(i,1);
                timeHist(1,(i-1)) = time;

                % Import position log
                trans = transpose(simData.out.camPose.signals.values(1,1:3,i));
                trans = trans*1000; %Convert from m to mm
                trans_W = R_sim2W * trans; %Convert to world coord sys
             
                % Import rotation log
                yaw = simData.out.camPose.signals.values(1,4,i);
                pitch = simData.out.camPose.signals.values(1,5,i);
                roll = simData.out.camPose.signals.values(1,6,i);

                %Convert euler angles to rotation matrix
                eul = [yaw pitch roll];
                R_sim = eul2rotm(eul,"ZYX");

                %Convert sim-camera coords to more conventional system
                %Sim Camera has z-up, x-forward. General model has
                %z-forward, y-down
                R_simCam2genCam = [0  0 1;
                                  -1  0 0;
                                   0 -1 0];
                R_W =R_sim2W*R_sim*R_simCam2genCam;


                %build Rt history
                rtHist(1:3,1:3,i-1) = R_W;
                rtHist(1:3,4,i-1) = trans_W;
                poseHist(:,i-1) = ([trans_W; yaw; pitch; roll]);
            end
        end


        %------------------------------------------------------------% 

        function convertVideo(sourceFile, targetFolder)
            %Convert video into series of frames. Name the frames according
            %to the time elapsed from video start. 
            
            %read video
            vid = VideoReader(sourceFile);

            %determine video parameters
            numFrames = vid.NumFrames;
            vidHeight = vid.Height;
            vidWidth = vid.Width;
            vidDuration = vid.Duration;
            fps = numFrames/vidDuration;
            secPerFrame = 1/fps;

            %frame-by-frame, convert frames to images and save
            for f=1:numFrames
                frame = read(vid, f); %read frame
                t=(f-1)*secPerFrame*1000; %get time in ms
                t = num2str(t);
                %frame_g = rgb2gray(frame_rgb); %convert to grayscale
                
                %save image to target folder
                while strlength(t)<4
                    t=strcat("0",t);
                end
              
                outputBaseFileName=strcat(t, ".jpg");
                %outputBaseFileName = sprintf('-%4.4d.jpg', t);
                outputFullFileName = fullfile(targetFolder, outputBaseFileName); %output filename
                imwrite(frame, outputFullFileName, 'jpg'); %write output file
            end

        end
        %------------------------------------------------------------%
            

        function [I_seq, I_seq_t] = importImageSeq(imgFolder)
            %Function to import images from a folder into a big 3D array.
            %Also outputs the time (in ms), from the video start, to the
            %respective frame. 

            imgFiles_ds = fileDatastore(imgFolder, 'ReadFcn', @importdata, "FileExtensions",".jpg");
            imgNames = imgFiles_ds.Files;
            numImgs = length(imgNames);

            %Define output variables
            I_seq = zeros(1,1,1);
            I_seq_t = zeros(1,numImgs);

            %image-by-image
            for f=1:numImgs
                I_rgb = imread(string(imgNames(f))); %read image into array
                I_g = rgb2gray(I_rgb);%convert to grayscale
                if f==1
                    I_seq = I_g;
                else
                    I_seq(:,:,f)=I_g; %Put into big array
                end
                
                %Get time
                [I_filepath, I_name, I_ext] = fileparts(imgNames(f)); 
                I_seq_t(f) = str2double(I_name)/1000; %in secs
                
            end
        end

        %------------------------------------------------------------%

        function I_annotated = markDetectedCheckers(I, x_pnts_i)
            %Add dots to show detected corners. Also label top-left point.

            I_annotated = I;
            
            %Mark corners with circles
            I_annotated = insertMarker(I_annotated, transpose(x_pnts_i(:,2:end)), "o");

            %Except for top left corner, which is marked with a square
            I_annotated = insertMarker(I_annotated, transpose(x_pnts_i(:,1)), "s");

        end



    end
end
