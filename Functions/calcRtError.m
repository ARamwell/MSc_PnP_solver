function resultStruct = calcRtError(resultStruct, Rt_hist_true, time_hist_true)

    methodsUsed = fieldnames(resultStruct);

    for m = 1: length(methodsUsed)

        methodName = methodsUsed{m};
        
        currentTime = resultStruct.(methodName).time(1,end);

        [~, closestIndex] = min(abs(time_hist_true-currentTime));

        Rt_true = Rt_hist_true(1:3, 1:4, closestIndex);
        Rt_calc = resultStruct.(methodName).Rt(1:3, 1:4, end);

        error = p3pFuncs.getOrientErrArr(Rt_true, Rt_calc, currentTime);

        resultStruct.(methodName).error(1:3, end+1) = error;


        



end