classdef worldObject
    %MAPOBJECT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Position
        Orientation
        Dimensions
        Actor
    end
    
    methods
        function obj = worldObject(position, orientation, dimensions)
            %MAPOBJECT Construct an instance of this class
            %   Detailed explanation goes here
            
            obj.Position = position;
            obj.Orientation = orientation;
            obj.Dimensions = dimensions; %x, y, z (object frame)


            
        end
    end
    methods (Static)
        function Rt_A2S = calcTransformA2S(Rt_B2W, Rt_A2B)

            %Define transformation between world and simulation. basically,
            %just making it right-handed.
            h = [0 0 0 1]; %homogeniser
            R_W2S = [1, 0,  0;
                     0, -1, 0;
                     0, 0,  1];
            t_W2S = [0; 0; 0];
            Rt_W2S = [R_W2S t_W2S];
            Rt_W2S_h =[Rt_W2S; h];

            Rt_B2W_h = [Rt_B2W; h];
            Rt_A2B_h = [Rt_A2B; h];

            %Get Rt_A2S
            Rt_A2S_h = Rt_W2S_h * Rt_B2W_h * Rt_A2B_h;
            Rt_A2S = Rt_A2S_h(1:3, 1:4);

        end    

        function Rt_B2W = calcTransformB2W(orientation, position)
            R_B2W = eul2rotm(deg2rad(orientation), 'XYZ'); %Convert euler angles to rotation matrix
            t_B2W = transpose(position);
            Rt_B2W = [R_B2W t_B2W];
        end
        % function outputArg = method1(obj,inputArg)
        %     %METHOD1 Summary of this method goes here
        %     %   Detailed explanation goes here
        %     outputArg = obj.Property1 + inputArg;
        % end
    end
end

