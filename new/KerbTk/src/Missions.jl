module Missions

using ..Arenas: Arena
using ..Bodies: SolarSystem
using ..Vessels: VesselClassId, VesselClass

using MsgPack: MsgPack

mutable struct Mission
    system::SolarSystem
    classes::Arena{VesselClassId,VesselClass}

    Mission() = new(SolarSystem(Dict()), Arena{VesselClassId,VesselClass}())
    function Mission(system::SolarSystem, classes::Arena{VesselClassId,VesselClass})
        return new(system, classes)
    end
end

MsgPack.msgpack_type(::Type{Mission}) = MsgPack.StructType()

end
