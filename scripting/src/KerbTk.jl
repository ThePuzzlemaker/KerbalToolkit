module KerbTk

using StaticArrays

export UT, Orbits, Bodies, Missions, StateVectors

struct UT
    # Universal time as seconds
    # TODO: consider using integer representation
    inner::Float64
end

Base.convert(::Type{UT}, x::Float64) = UT(x)
Base.convert(::Type{Float64}, x::UT) = x.inner

include("Support.jl")
include("Orbits.jl")
include("Bodies.jl")
include("SolarSystems.jl")
include("Missions.jl")
include("StateVectors.jl")

import REPL

global ktk_term = nothing
global ktk_repl = nothing

using .Orbits: Orbit
using .Bodies: Body
using .SolarSystems: SolarSystem
using .Missions: Mission, mission, load_mission, save_mission
using .StateVectors:
    StateVector, ReferenceFrame, BodyCenteredInertial, BodyCenteredBodyFixed, sv_bci

export Orbit,
    Body,
    SolarSystem,
    Mission,
    mission,
    StateVector,
    ReferenceFrame,
    BodyCenteredInertial,
    BodyCenteredBodyFixed,
    load_mission,
    save_mission,
    sv_bci

function init()
    Missions.init_mission()
end

function run_repl()
    global ktk_term = REPL.Terminals.TTYTerminal(
        get(ENV, "TERM", Sys.iswindows() ? "" : "dumb"),
        stdin,
        stdout,
        stderr,
    )
    global ktk_repl = REPL.LineEditREPL(ktk_term, true)
    while true
        REPL.run_repl(ktk_repl)
    end
end

end # module KerbTk
