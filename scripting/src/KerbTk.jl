module KerbTk

using StaticArrays

export UT, Orbits, bodies, init_repl

struct UT
    # Universal time as seconds
    # TODO: consider using integer representation
    inner::Float64
end

Base.convert(::Type{UT}, x::Float64) = UT(x)
Base.convert(::Type{Float64}, x::UT) = x.inner

include("Orbits.jl")
include("Bodies.jl")


# struct SolarSystem
#     bodies::Dict{String, Body}
# end
import REPL

global ktk_term = nothing
global ktk_repl = nothing

function init_repl()
    global ktk_term = REPL.Terminals.TTYTerminal(get(ENV, "TERM", Sys.iswindows() ? "" : "dumb"), stdin, stdout, stderr)
    global ktk_repl = REPL.LineEditREPL(ktk_term, true)
    REPL.run_repl(ktk_repl, backend_on_current_task = false)
end

# function run_repl()
    
# end

end # module KerbTk
