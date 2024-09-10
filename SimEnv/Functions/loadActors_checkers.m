function loadActors_checkers(Actor, World)
%LOADMAPACTORS Function to populate Simulink "Simulation 3D actor block"
%with wall objects defined in .mat file. 
%   Detailed explanation goes here

    %Load .mat file
    load('C:\Users\alyss\OneDrive - University of Cape Town\Sandbox\PnP\Pnp_solver\GitClone\Resources\map');
    
    if isempty(worldObjectStruct.checkers) ~= true
        %For each checkerboard defined in struct
        for i=1:size(worldObjectStruct.checkers, 2)
    
            NewActor = worldObjectStruct.checkers(1,i).Actor;
            add(World, NewActor);
            NewActor.Parent = Actor;    %New actor must be parent of block's main actor
        end
    end
   
end

