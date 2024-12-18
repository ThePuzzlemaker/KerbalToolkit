"""
Structures and functions dealing with missions and mission plans.
"""
module Missions

using Match

using ..Support: @check
using ..SolarSystems: SolarSystems, SolarSystem

export Opaque, Mission, init_mission, mission, load_mission, save_mission

abstract type Opaque end

"""
A mission.

The mission is the central datastructure of KerbalToolkit and holds
all saved information.

# Fields

- `system::SolarSystem`: Reference solar system.
"""
mutable struct Mission
    _inner::Ptr{Opaque}
end

"""
    load_mission(file::String)

Replace the global mission by loading one from the provided `file`.
"""
function load_mission(file::String)
    @check ccall(ktk_missions_load, Nothing, (Ptr{Opaque}, String), ktk_mission_ptr, file)
end

"""
    save_mission(file::String)

Save the global mission to the provided `file`.
"""
function save_mission(file::String)
    @check ccall(ktk_missions_save, Nothing, (Ptr{Opaque}, String), ktk_mission_ptr, file)
end

function Base.getproperty(mission::Mission, s::Symbol)
    return @match s begin
        :system => SolarSystem(
            ccall(
                ktk_missions_get_system,
                Ptr{SolarSystems.Opaque},
                (Ptr{Opaque},),
                getfield(mission, :_inner),
            ),
            mission,
        )
        s => getfield(mission, s)
    end
end

global ktk_missions_get_system = nothing
global ktk_missions_load = nothing
global ktk_missions_save = nothing

global ktk_mission_ptr = nothing

"""
    mission::Mission

The global mission singleton.
"""
global mission::Mission = Mission(Ptr{Opaque}(0))

function Base.show(io::IO, ::Mission)
    write(io, "Mission(…)")
end

function init_mission()
    mission._inner = ktk_mission_ptr
end

end # KerbTk.Missions
