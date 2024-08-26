classdef p3pFuncs
    methods (Static)

        %------------------------------------------------------------%
        function x_pnt_c_unit = getCameraVector(K, x_pnt_i)        
            %Function to get unit vector describing the ray between a calibrated
            %camera's centre of perspective (CP) and the position of a point on the
            %image plane (i). The point x_pnt_i should be a 2x1 column vector of the
            %form [u; v], where u is the "x" coordinate (horizontal, right is positive)
            %and v is the "y" coordinate (vertical, down is positive); the origin of
            %the image is the top left corner. 

            x_pnt_c_unit = zeros(3,1);
        
            for j=1:size(x_pnt_i, 2)
                x_pnt_i_aug = [x_pnt_i(:,j); 1];     %Augment vector to homogenise
                K_inv = inv(K);
                
                x_pnt_c = K_inv * x_pnt_i_aug;      %Times by inverse of K. Apparently divide is faster.
                
                x_pnt_c_unit(:,j) = x_pnt_c / vecnorm(x_pnt_c);

            end
        
        end
        
        %------------------------------------------------------------%
        function L_ab = calcAngle(A, B)
        
            %Calculate angle between two vectors.
            
            L_ab = acos(dot(A,B)/(norm(A)*norm(B)));
        
        end
                
        %------------------------------------------------------------%
        function [Rt_CW] = findTransformation_SVD(X_ABC_W, X_ABC_C)
            
            P = X_ABC_W;
            P_dash = X_ABC_C;
                    
            %Find the mean of the columns (i.e. the centroid of the shape)
            P_centroid = mean(P,2);
            P_dash_centroid = mean(P_dash, 2);
            %Centre each vector
            Q = P - P_centroid;  
            Q_dash = P_dash - P_dash_centroid;
        
            %Find the covariance matrix
            H = Q_dash * transpose(Q);
                
            %Perform SVD (singular value decomposition)
            [U, S, V] = svd(H);
        
            % %Calculate rotation matrix
            R = V * transpose(U);
            %Correct sign if needed
            if det(R)<0
                V(:,end) = -1 * V(:,end);
                %and recompute R
                R = V * transpose(U);
            end
            
            t = P_centroid - R*P_dash_centroid;

            %The above solution gives the R & t from the Camera -> World.
           
            Rt_CW = [R t];
            %Rt_WC = p3pFuncs.invertRt(Rt);

        
        end

        %------------------------------------------------------------%

        function [x_ABCD_i, X_ABCD_W] = checkerOuterCornerSelector(x_pnts_i, X_pnts_W, boardHeight, boardWidth)
            %Expects points to be imported as column vectors. 

            A_index = 1;
            B_index = boardHeight - 1;
            C_index = ((boardHeight - 1) * (boardWidth - 2)) + 1;
            D_index = ((boardHeight - 1) * (boardWidth - 1)) ;

            %Obtain pixel positions
            x_ABCD_i = [x_pnts_i(:, A_index) x_pnts_i(:,B_index) x_pnts_i(:,C_index) x_pnts_i(:,D_index)];
            X_ABCD_W = [X_pnts_W(:,A_index) X_pnts_W(:,B_index) X_pnts_W(:,C_index) X_pnts_W(:,D_index)];
        end
        %------------------------------------------------------------%

        function err = calcReprojErrorWC(K, x_i, X_W, Rt_WC)

            %Initialise output variables
            x_i_star = zeros(2,1);

            %Extract input variables
            R_WC = Rt_WC(1:3, 1:3);
            t_WC = Rt_WC(1:3, 4);           

            
            %Project the world point into the camera frame
            x_c_star = K*((R_WC * X_W) + t_WC);

            %Normalise by the 3rd dimension to get the pixel coords
            x_i_star(1,1) = x_c_star(1,1)/norm(x_c_star(3,1)); 
            x_i_star(2,1) = x_c_star(2,1)/norm(x_c_star(3,1));

            %Calculate reproj error
            err = norm(x_i - x_i_star);
            %err = norm(x_c - x_c_star);
        end
    
        %------------------------------------------------------------%

        function err = calcReprojErrorCW(K, x_i, X_W, Rt_CW)

            %Extract input variables
            R_CW = Rt_CW(1:3, 1:3);
            t_CW = Rt_CW(1:3, 4);
            
            %Project the image point into the camera frame
            x_i_aug = [x_i; 1];
            x_c_star = inv(K) * x_i_aug;
            x_c_star_unit = x_c_star/vecnorm(x_c_star);

            %Project the camera point into the world frame
            X_W_star = (R_CW * x_c_star_unit) + t_CW;

            %Calculate reproj error
            err = norm(X_W - X_W_star);

        end
    
        %------------------------------------------------------------%

        function [bestRt, minErr] = chooseRtWithMinReprojErrorWC(K, Rt_arr, x_pnt_i, X_pnt_W)

            %Initialise variables
            minErr = 10000000;%Arbitrarily large
            bestRt = Rt_arr(:,:,1);
        
            %For each Rt in the array
            for j=1:size(Rt_arr, 3)

                Rt = Rt_arr(:,:,j);

                err_j = p3pFuncs.calcReprojErrorWC(K, x_pnt_i, X_pnt_W, Rt);

                %If new Rt has lower reproj error, choose it
                if err_j < minErr
                    minErr = err_j;
                    bestRt = Rt;
                end
            end
        end

        %------------------------------------------------------------%

        function [bestRt, minErr] = chooseRtWithMinReprojErrorCW(K, Rt_arr, x_pnt_i, X_pnt_W)

            %Initialise variables
            minErr = 10000;%Arbitrarily large
            bestRt = Rt_arr(:,:,1);
        
            %For each Rt in the array
            for j=1:size(Rt_arr, 3)

                Rt = Rt_arr(:,:,j);

                err_j = p3pFuncs.calcReprojErrorCW(K, x_pnt_i, X_pnt_W, Rt);

                %If new Rt has lower reproj error, choose it
                if err_j < minErr
                    minErr = err_j;
                    bestRt = Rt;
                end
            end
        end

        %------------------------------------------------------------%

        function [bestRt, mostInliers] = chooseRtWithMostInliersWC(K, Rt_arr, inlierThreshold, x_pnts_i, X_pnts_W)

            %Initialise variables
            bestRt = Rt_arr(:,:,1);
            mostInliers = 0;
            

            %For each Rt in the array
            for j=1:size(Rt_arr, 3)

                Rt_j = Rt_arr(:,:,j);
                numInliers_j = 0;

                %Calculate reproj error for each set of points
                for n=1:size(x_pnts_i,2)

                    x_n_i = x_pnts_i(:,n);
                    X_n_W = X_pnts_W(:,n);

                    err_n = p3pFuncs.calcReprojErrorWC(K, x_n_i, X_n_W, Rt_j);
                    
                    %If reproj error is low enough, increment inlier count
                    if err_n <= inlierThreshold
                        numInliers_j = numInliers_j + 1;
                    end
                    %and move onto next point
                end

                %check if this Rt is better than the last best
                if numInliers_j > mostInliers
                    mostInliers = numInliers_j;
                    bestRt = Rt_j;
                end
            end
        end

        %------------------------------------------------------------%
        
        function [bestRt, mostInliers] = chooseRtWithMostInliersCW(K, Rt_arr, inlierThreshold, x_pnts_i, X_pnts_W)

            %Initialise variables
            bestRt = Rt_arr(:,:,1);
            mostInliers = 0;
            

            %For each Rt in the array
            for j=1:size(Rt_arr, 3)

                Rt_j = Rt_arr(:,:,j);
                numInliers_j = 0;

                %Calculate reproj error for each set of points
                for n=1:size(x_pnts_i,2)

                    x_n_i = x_pnts_i(:,n);
                    X_n_W = X_pnts_W(:,n);

                    err_n = p3pFuncs.calcReprojErrorCW(K, x_n_i, X_n_W, Rt_j);
                    
                    %If reproj error is low enough, increment inlier count
                    if err_n <= inlierThreshold
                        numInliers_j = numInliers_j + 1;
                    end
                    %and move onto next point
                end

                %check if this Rt is better than the last best
                if numInliers_j > mostInliers
                    mostInliers = numInliers_j;
                    bestRt = Rt_j;
                end
            end
        end
        %------------------------------------------------------------%
        
        function Rt_inv = invertRt(Rt)

            R = Rt(1:3, 1:3);
            t = Rt(1:3, 4);

            R_inv = inv(R);
            t_inv = -1 * R_inv * t;

            Rt_inv = [R_inv t_inv];
        end

        %------------------------------------------------------------%


        %------------------------------------------------------------%

        function test()
            disp('Hello World');
        end

         %------------------------------------------------------------%

        function X_pnts_W = calcCheckerEdgeCoords_W(checkerSize, squareEdgeLength, Rt_B2W)
        
            boardWidth = checkerSize(2);
            boardHeight = checkerSize(1);
            X_pnts_W = zeros(3,(boardWidth-1)*(boardHeight-1));
            X_pnts_B = zeros(3,(boardWidth-1)*(boardHeight-1));
        
            R_B2W = Rt_B2W(1:3,1:3);
            t_B2W = Rt_B2W(1:3,4);
        
            %for each column
            for c=1 : boardWidth-1
                for r = 1: boardHeight-1
                    x = (c-1)*squareEdgeLength;
                    y = (r-1)*squareEdgeLength;
                    z = 1;
                    X_pnts_B(1,  (((c-1)*(boardHeight-1)) + r)) = x;
                    X_pnts_B(2,  (((c-1)*(boardHeight-1)) + r)) = y;
                    X_pnts_B(3,  (((c-1)*(boardHeight-1)) + r)) = z;
                end
            end

            %For each point
            for i=1:size(X_pnts_B,2)
                X_B = X_pnts_B(:,i); 
                X_pnts_W(:,i) = (R_B2W * X_B) + t_B2W;
            end
        end
        %------------------------------------------------------------%

        function [rotErr, posErr] = calcOrientError(Rt_actual, Rt_calc)
            
            %Extract useful variables
            R_act = Rt_actual(1:3,1:3);
            t_act = Rt_actual(1:3,4);
            R_calc = Rt_calc(1:3,1:3);
            t_calc =Rt_calc(1:3,4);

            %-----------------------%
            %Calculate rotation error using quaternion representations
            
            %First, convert to quaternions
            quat_B2W_act = quaternion(rotm2quat(R_act));
            quat_W2B_calc =quaternion(rotm2quat(transpose(R_calc)));

            %Then, calculate quaternion representing the actual frame in
            %the body frame. This represents the rotation between the two
            %frames.
            quat_act2calc = quat_B2W_act * quat_W2B_calc;
            quat_act2calc_arr = compact(quat_act2calc);

            %Extract error angle. Could also get rotation axis.
            rotErr_angle = 2*acos(quat_act2calc_arr(1));

            %Define rotational error
            rotErr =rotErr_angle;

            %-----------------------%
            %Calculate position error using euclidean distance
            
            posErr = norm(t_act - t_calc);
            

        end 

        %------------------------------------------------------------%

        function orientErrArr = getOrientErrArr(Rt_act, Rt_calc, time)
            
            [rotErr, posErr] = p3pFuncs.calcOrientError(Rt_act, Rt_calc);

            orientErrArr = [time; rotErr; posErr];


        end

    end
    
end