module Missions

using Match

using ..Support: ReadLock, WriteLock, OpaqueLock
using ..SolarSystems: SolarSystems, SolarSystem

export OpaqueOuter, OpaqueInner, LockedMission, Mission, init_mission, mission

abstract type OpaqueOuter end
abstract type OpaqueInner end

mutable struct LockedMission
    _inner::Ptr{OpaqueInner}
    _parent::Any
    _readonly::Bool
    # Placeholders for IDE's sake
    system::SolarSystem
    function LockedMission(_inner::Ptr{OpaqueInner}, _parent::Any, _readonly::Bool)
        new(_inner, _parent, _readonly)
    end
end

function Base.show(io::IO, mission::LockedMission)
    write(io, """
LockedMission(
    system=$(repr(mission.system))
)""")
end

Base.deepcopy_internal(::LockedMission, ::IdDict) = error("Not implemented")

function Base.getproperty(mission::LockedMission, s::Symbol)
    return @match s begin
        :system => SolarSystem(ccall(ktk_missions_get_system, Ptr{SolarSystems.Opaque}, (Ptr{OpaqueInner},), getfield(mission, :_inner)), mission)
    end
end

mutable struct Mission
    _inner::Ptr{OpaqueOuter}
    read::ReadLock{LockedMission, OpaqueInner}
    write::WriteLock{LockedMission, OpaqueInner}
end

global ktk_missions_get_data_ptr = nothing
global ktk_missions_get_lock_ptr = nothing
global ktk_missions_get_system   = nothing

global ktk_mission_ptr = nothing
global mission = nothing

function Base.show(io::IO, ::Mission)
    write(io, "Mission(<locked>)")
end

function init_mission()
    data_ptr = ccall(ktk_missions_get_data_ptr, Ptr{OpaqueInner}, (Ptr{OpaqueOuter},), ktk_mission_ptr)
    lock_ptr = ccall(ktk_missions_get_lock_ptr, Ptr{OpaqueLock}, (Ptr{OpaqueOuter},), ktk_mission_ptr)

    read = ReadLock{LockedMission, OpaqueInner}(data_ptr, lock_ptr, nothing, false)
    write = WriteLock{LockedMission, OpaqueInner}(data_ptr, lock_ptr, nothing, false)
    
    global mission = Mission(ktk_mission_ptr, read, write)
    mission.read._parent = mission
    mission.write._parent = mission
end

end # KerbTk.Missions
