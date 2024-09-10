function loadActors_walls(Actor, World)
%LOADMAPACTORS Function to populate Simulink "Simulation 3D actor block"
%with wall objects defined in .mat file. 
%   Detailed explanation goes here

    load('C:\Users\alyss\OneDrive - University of Cape Town\Sandbox\PnP\Pnp_solver\GitClone\Resources\map');
    
    if isempty(worldObjectStruct.walls) ~= true
        for i=1:size(worldObjectStruct.walls, 2)
    
            NewActor = worldObjectStruct.walls(1,i).Actor;%sim3d.Actor('ActorName', mapStruct.walls(1,i).A)
            add(World, NewActor);
            NewActor.Parent = Actor;
        end
    end
   
end

