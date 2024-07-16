function A = builtinP3P(imagePts_vert, worldPts_vert, focalLngth, principalPnt, imageSize, maxReprojError, estTranslationVec, estRotationMat, camPlot)
    
    intr = cameraIntrinsics(focalLngth, principalPnt, imageSize);

    worldPose = estworldpose(imagePts_vert, worldPts_vert, intr);

    A = worldPose.A;
    R_or = worldPose.R;
    t_or = transpose(worldPose.Translation);
    
    R_inv = transpose(R_or);
    t_inv = - R_inv * t_or;
    A_inv = [R_inv t_inv];
    A_inv = [A_inv; 0 0 0 1];

    if camPlot == 1
        figure;
        pcshow(worldPts_vert,VerticalAxis="Y",VerticalAxisDir="down", ...
        MarkerSize=30);
        
        hold on
        plotCamera(Size=10,Orientation=worldPose.R, ...
            Location=worldPose.Translation);
        hold on
        %plotCamera(Size=10,Orientation=estRotationMat, ...
        %    Location=estTranslationVec,Color=[0 1 0]);
        hold off
    end 



end