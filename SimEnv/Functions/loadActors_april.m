function loadActors_april(Actor, World)
%LOADMAPACTORS Function to populate Simulink "Simulation 3D actor block"
%with wall objects defined in .mat file. 
%   Detailed explanation goes here

    %Load .mat file
    load('C:\Users\alyss\OneDrive - University of Cape Town\Sandbox\PnP\Pnp_solver\GitClone\Resources\map');
    
    %For each checkerboard defined in struct
    for i=1:size(worldObjectStruct.aprils, 2)

        NewActor = worldObjectStruct.aprils(1,i).Actor;
        add(World, NewActor);
        NewActor.Parent = Actor;    %New actor must be parent of block's main actor
    end
   
end

