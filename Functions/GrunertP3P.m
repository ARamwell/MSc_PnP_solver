function Rt = GrunertP3P (X_ABC_W, x_ABC_c_unit)
%Conducts Grunert's P3P, given the positions of three points in the world
%frame and the vectors of the corresponding points, from the centre of 
%perspective of a calibrated camera to the image plane (i.e. homogenised).
%Both should be given in column vector form. 
    
    %% Declare output variable
    Rt = [];

    %% Extract some variables

    %World points
    A_W = X_ABC_W(:,1);
    B_W = X_ABC_W(:,2);
    C_W = X_ABC_W(:,3);

    %Projection rays
    f_a = x_ABC_c_unit(:,1);
    f_b = x_ABC_c_unit(:,2);
    f_c = x_ABC_c_unit(:,3);

    %Calculate the lengths of the "World" triangle
    l_AB = norm(B_W - A_W);
    l_BC = norm(C_W - B_W);
    l_AC = norm(C_W - A_W);

    %Calculate the angles between the projection rays
    theta_ab = p3pFuncs.calcAngle(f_a, f_b);
    theta_bc = p3pFuncs.calcAngle(f_b, f_c);
    theta_ac = p3pFuncs.calcAngle(f_a, f_c);

    %% Assign to variables matching C. Stachniss lectures

    X1 = A_W;
    X2 = B_W;
    X3 = C_W;

    f1 = f_a;
    f2 = f_b;
    f3 = f_c;

    c = l_AB;
    a = l_BC;
    b = l_AC;

    gamma = theta_ab;
    alpha = theta_bc;
    beta = theta_ac;

    %% Set up polynomial
    %Polynomial is in terms of v, where v = s3/s1. u = s2/s1.

    %Calculate constants
    a2minc2ovb2 =  (a^2 - c^2) / b^2;
    a2plusc2ovb2 =  (a^2 + c^2) / b^2; 

    A0 = (1 + a2minc2ovb2)^2   -   4*((a*cos(gamma)/b)^2);
    A1 = - a2minc2ovb2*(1 + a2minc2ovb2)*cos(beta)   +  2 * ((a*cos(gamma)/b)^2)*cos (beta) - (1 - a2plusc2ovb2) * cos(alpha) * cos(gamma);
    A1 = 4 * A1;
    A2 = a2minc2ovb2^2   -   1   +   2*((a2minc2ovb2 * cos(beta))^2)  +  2 *(  ((b^2 - c^2)/(b^2))  * (cos(alpha))^2  )   - 4*a2plusc2ovb2*cos(alpha)*cos(beta)*cos(gamma)   +   2*((b^2 - a^2)/(b^2))*((cos(gamma))^2);
    A2 = 2* A2;
    A3 = a2minc2ovb2 * (1 - a2minc2ovb2) * cos(beta)    -   (1 - a2plusc2ovb2) * cos(alpha) * cos(gamma)    +   2 * ((c * cos(alpha)/b)^2) * cos(beta);
    A3 = 4 * A3;
    A4 = (a2minc2ovb2 -1)^2   - (2 * c * cos(alpha) / b)^2;

    p = [A4 A3 A2 A1 A0];
    
    %% Solve polynomial

    h = roots(p);

    %Choose only real roots
    h = h(imag(h)==0);
    h = real(h);

    %% Get full ray lengths
    %Remember, polynomial solves for v, where v = s3/s1

    %For each possible root v
    for i = 1:numel(h)

        v = h(i);

        %Find length s1
        s1 = sqrt ( (b^2) / ((1 + v^2) - 2*v*cos(beta)));

        %Find length s3
        s3 = v*s1;

        %Find length s2: using quadratic
        G2 = 1;
        G1 = -2*s3*cos(alpha);
        G0 = (s3^2 - a^2);
        g = [G2 G1 G0];
        s2 = roots(g);
        s2 = s2(imag(s2)==0);
        s2 = s2(s2>=0);  %Can't be a negative length
        
        for j=1:size(s2,1)
            %Compute 3D coords of control points in camera coord system
            X1_C = s1 * f1;
            X2_C = s2(j) * f2;
            X3_C = s3 * f3;
    
            %Calculate transformation
            [Rt_ij] = p3pFuncs.findTransformation_SVD([X1 X2 X3], [X1_C X2_C X3_C]);
    
            %Add to output matrix
            if i == 1
                Rt(:,:,1) = [Rt_ij];
            else
                Rt(:,:,end+1) = [Rt_ij];
            end
        end

    end
end





        




        





