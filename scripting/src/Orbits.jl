module Orbits

using Match
import ..UT

export Opaque, Orbit

abstract type Opaque end

"""
    Orbit(p, e, i, lan, argpe, epoch, ta)

An orbit, as described by Keplerian elements.

# Fields

- `p::Float64`:     Semi-latus rectum (`km`).
- `e::Float64`:     Eccentricity (dimensionless).
- `i::Float64`:     Inclination (`rad`).
- `lan::Float64`:   Longitude of ascending node (`rad`).
- `argpe::Float64`: Argument of periapsis (`rad`).
- `epoch::UT`:      Epoch at true anomaly ([Universal Time](@ref UT)).
- `ta::Float64`:    True anomaly (`rad`).
"""
# Implementation details:
# - Box::into_raw -or- stack-pointer-to
#   + thus, _parent is used as a keepalive in SP cases
mutable struct Orbit
    _inner::Ptr{Opaque}
    _parent::Any
    _readonly::Bool
    # Placeholders for IDE's sake
    p::Float64
    e::Float64
    i::Float64
    lan::Float64
    argpe::Float64
    epoch::UT
    ta::Float64
    

    function Orbit(p, e, i, lan, argpe, epoch, ta)
        _inner = ccall(ktk_orbit_new,
                       Ptr{Opaque},
                       (Float64, Float64, Float64,
                        Float64, Float64, Float64, Float64),
                       convert(Float64, p),
                       convert(Float64, e),
                       convert(Float64, i),
                       convert(Float64, lan),
                       convert(Float64, argpe),
                       convert(Float64, epoch),
                       convert(Float64, ta))
        x = new(_inner, nothing, false)
        function f(t)
            if getfield(t, :_inner) == Ptr{Opaque}(0)
                @async println("Warning: Tried to destroy an invalidated Orbit")
            else
                ccall(ktk_orbit_free, Nothing, (Ptr{Opaque},), t._inner)
                t._inner = Ptr{Opaque}(0)
            end
        end
        finalizer(f, x)
    end

    function Orbit(_inner::Ptr{Opaque}, _parent::Any, _readonly::Bool)
        new(_inner, _parent, _readonly)
    end
end

function Base.show(io::IO, obt::Orbit)
    write(io, "Orbit($(obt.p), $(obt.e), $(obt.i), $(obt.lan), $(obt.argpe), $(repr(obt.epoch)), $(obt.ta))")
end

function Base.deepcopy_internal(obt::Orbit, ::IdDict)
    return Orbit(obt.p, obt.e, obt.i, obt.lan, obt.argpe, obt.epoch, obt.ta)
end

global ktk_orbit_get_p     = nothing
global ktk_orbit_get_e     = nothing
global ktk_orbit_get_i     = nothing
global ktk_orbit_get_lan   = nothing
global ktk_orbit_get_argpe = nothing
global ktk_orbit_get_epoch = nothing
global ktk_orbit_get_ta    = nothing

global ktk_orbit_set_p     = nothing
global ktk_orbit_set_e     = nothing
global ktk_orbit_set_i     = nothing
global ktk_orbit_set_lan   = nothing
global ktk_orbit_set_argpe = nothing
global ktk_orbit_set_epoch = nothing
global ktk_orbit_set_ta    = nothing

global ktk_orbit_new  = nothing
global ktk_orbit_free = nothing

function Base.getproperty(obt::Orbit, s::Symbol)
    if getfield(obt, :_inner) == Ptr{Opaque}(0) && s != :_inner
        error("Tried to access field $(s) on an invalidated Orbit")
    end
    return @match s begin
        :p     => ccall(ktk_orbit_get_p, Float64, (Ptr{Opaque},), getfield(obt, :_inner))
        :e     => ccall(ktk_orbit_get_e, Float64, (Ptr{Opaque},), getfield(obt, :_inner))
        :i     => ccall(ktk_orbit_get_i, Float64, (Ptr{Opaque},), getfield(obt, :_inner))
        :lan   => ccall(ktk_orbit_get_lan, Float64, (Ptr{Opaque},), getfield(obt, :_inner))
        :argpe => ccall(ktk_orbit_get_argpe, Float64, (Ptr{Opaque},), getfield(obt, :_inner))
        :epoch => UT(ccall(ktk_orbit_get_epoch, Float64, (Ptr{Opaque},), getfield(obt, :_inner)))
        :ta    => ccall(ktk_orbit_get_ta, Float64, (Ptr{Opaque},), getfield(obt, :_inner))
        s      => getfield(obt, s)
    end
end

function Base.setproperty!(obt::Orbit, s::Symbol, val::Any)
    if getfield(obt, :_inner) == Ptr{Opaque}(0) && s != :_inner
        error("Tried to set field $(s) on an invalidated Orbit")
    end
    if getfield(obt, :_readonly)
        error("Tried to set field $(s) on a readonly Orbit")
    end
    @match s begin
        :p     => ccall(ktk_orbit_set_p, Nothing, (Ptr{Opaque}, Float64), getfield(obt, :_inner), convert(Float64, val))
        :e     => ccall(ktk_orbit_set_e, Nothing, (Ptr{Opaque}, Float64), getfield(obt, :_inner), convert(Float64, val))
        :i     => ccall(ktk_orbit_set_i, Nothing, (Ptr{Opaque}, Float64), getfield(obt, :_inner), convert(Float64, val))
        :lan   => ccall(ktk_orbit_set_lan, Nothing, (Ptr{Opaque}, Float64), getfield(obt, :_inner), convert(Float64, val))
        :argpe => ccall(ktk_orbit_set_argpe, Nothing, (Ptr{Opaque}, Float64), getfield(obt, :_inner), convert(Float64, val))
        :epoch => ccall(ktk_orbit_set_epoch, Nothing, (Ptr{Opaque}, Float64), getfield(obt, :_inner), convert(Float64, val))
        :ta    => ccall(ktk_orbit_set_ta, Nothing, (Ptr{Opaque}, Float64), getfield(obt, :_inner), convert(Float64, val))
        s      => setfield!(obt, s, val)
  end
end

end # KerbTk.Orbits
