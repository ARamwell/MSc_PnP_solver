function [outputArg1,outputArg2] = poseHist_update(poseHist, newPose, newTime)

    poseMethods = fieldnames(newPose);

    for m=1: length(poseMethods)
        methodName = poseMethods{m};

        poseHist.(methodName) = [newTime, newPose]

    if isempty(poseHist)



        

    %poseHist is struct


end

