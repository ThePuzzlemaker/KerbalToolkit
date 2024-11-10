module Bodies

using Match
using StaticArrays


import ..Orbits
import ..Orbits.Orbit

export Opaque, Body

abstract type Opaque end

"""
A celestial body.

This type has no public constructors, nor any setters as it is
expected that body information is obtained from the current solar
system.

# Fields

- `mu::Float64`: Standard gravitational parameter (`km³/s²`).
- `radius::Float64`: Mean radius of the body's sphere (`km`).
- `ephem::Orbit`: Ephemerides at starting epoch.
- `rotperiod::Float64`: Rotational period; length of sidereal day (`sec`).
- `rotini::Float64`: Initial rotation about the body spin axis at UT=0 (`rad`).
- `satellites::Vector{String}`: List of names of bodies orbiting this body.
- `parent::Union{String, Nothing}`: Name of the parent body of this body, if any.
- `name::String`: Name of this body as displayed in KSP.
- `is_star::Bool`: Is this a star?
- `soi::Float64`: Radius of this body's sphere of influence (`km`).
- `angvel::SVector{3, Float64}`: Angular momentum direction in Body-Centered
  Inertial coordinates.
"""
# Implementation details:
# - Always refcounted w/ Arc (no _parent) necessary
mutable struct Body
    _inner::Ptr{Opaque}
    # Placeholders for IDE's sake
    mu::Float64
    radius::Float64
    ephem::Orbit
    rotperiod::Float64
    rotini::Float64
    satellites::Vector{String}
    parent::Union{String, Nothing}
    name::String
    is_star::Bool
    soi::Float64
    angvel::SVector{3, Float64}

    function Body(_inner::Ptr{Opaque})
        x = new(_inner)
        function f(t)
            if getfield(t, :_inner) == Ptr{Opaque}(0)
                @async println("Warning: Tried to destroy an invalidated Body")
            else
                ccall(ktk_body_free, Nothing, (Ptr{Opaque},), t._inner)
                t._inner = Ptr{Opaque}(0)
            end
        end
        finalizer(f, x)
    end
end

function Base.show(io::IO, body::Body)
    write(io, """
Body(
    mu = $(repr(body.mu)),
    radius = $(repr(body.radius)),
    ephem = $(repr(body.ephem)),
    rotperiod = $(repr(body.rotperiod)),
    rotini = $(repr(body.rotini)),
    satellites = $(repr(body.satellites)),
    parent = $(repr(body.parent)),
    name = $(repr(body.name)),
    is_star = $(repr(body.is_star)),
    soi = $(repr(body.soi)),
    angvel = $(repr(body.angvel))
)""")
end

# function Base.copy(body::Body)
#     new_body = Body(body._inner)
#     ccall(ktk_body_increment_rc
# end

Base.deepcopy_internal(::Body, ::IdDict) = error("Not implemented")

global ktk_body_free         = nothing
global ktk_body_increment_rc = nothing

global ktk_body_get_mu         = nothing
global ktk_body_get_radius     = nothing
global ktk_body_get_ephem      = nothing
global ktk_body_get_rotperiod  = nothing
global ktk_body_get_rotini     = nothing
global ktk_body_get_satellites = nothing
global ktk_body_get_parent     = nothing
global ktk_body_get_name       = nothing
global ktk_body_get_is_star    = nothing
global ktk_body_get_soi        = nothing
global ktk_body_get_angvel     = nothing

global ktk_test_body = nothing

function Base.getproperty(body::Body, s::Symbol)
    if getfield(body, :_inner) == Ptr{Opaque}(0) && s != :_inner
        error("Tried to access field $(s) on an invalidated Body")
    end
    return @match s begin
        :mu => ccall(ktk_body_get_mu, Float64, (Ptr{Opaque},), getfield(body, :_inner))
        :radius => ccall(ktk_body_get_radius, Float64, (Ptr{Opaque},), getfield(body, :_inner))
        :ephem => Orbit(ccall(ktk_body_get_ephem, Ptr{Orbits.Opaque}, (Ptr{Opaque},), getfield(body, :_inner)), body, true)
        :rotperiod => ccall(ktk_body_get_rotperiod, Float64, (Ptr{Opaque},), getfield(body, :_inner))
        :rotini => ccall(ktk_body_get_rotini, Float64, (Ptr{Opaque},), getfield(body, :_inner))
        :satellites => ccall(ktk_body_get_satellites, Any, (Ptr{Opaque},), getfield(body, :_inner))
        :parent => ccall(ktk_body_get_parent, Any, (Ptr{Opaque},), getfield(body, :_inner))
        :name => ccall(ktk_body_get_name, String, (Ptr{Opaque},), getfield(body, :_inner))
        :is_star => ccall(ktk_body_get_is_star, Bool, (Ptr{Opaque},), getfield(body, :_inner))
        :soi => ccall(ktk_body_get_soi, Float64, (Ptr{Opaque},), getfield(body, :_inner))
        :angvel => ccall(ktk_body_get_angvel, Any, (Ptr{Opaque},), getfield(body, :_inner))
        s => getfield(body, s)
    end
end

function Base.setproperty!(body::Body, s::Symbol, val::Any)
    if getfield(body, :_inner) == Ptr{Opaque}(0) && s != :_inner
        error("Tried to set field $(s) on an invalidated Body")
    end
    @match s begin
        :_inner => setfield!(body, :_inner, val)
        s => error("Cannot set field $(s) on a Body")
    end
end

end # KerbTk.Bodies
