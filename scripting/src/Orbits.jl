"""
Orbits and associated parameters and operations.
"""
module Orbits

using Match
using ..Enums: Apsis, Apoapsis, Periapsis
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
mutable struct Orbit
    # Implementation details:
    # - Box::into_raw -or- stack-pointer-to
    #   + thus, _parent is used as a keepalive in SP cases
    _inner::Ptr{Opaque}
    _parent::Any
    _readonly::Bool

    function Orbit(p, e, i, lan, argpe, epoch, ta)
        _inner = ccall(
            ktk_orbit_new,
            Ptr{Opaque},
            (Float64, Float64, Float64, Float64, Float64, Float64, Float64),
            convert(Float64, p),
            convert(Float64, e),
            convert(Float64, i),
            convert(Float64, lan),
            convert(Float64, argpe),
            convert(Float64, epoch),
            convert(Float64, ta),
        )
        x = new(_inner, nothing, false)
        function f(t)
            if getfield(t, :_inner) == Ptr{Opaque}(0)
                error("Tried to destroy an invalidated Orbit")
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
    write(
        io,
        "Orbit($(obt.p), $(obt.e), $(obt.i), $(obt.lan), $(obt.argpe), $(repr(obt.epoch)), $(obt.ta))",
    )
end

function Base.deepcopy_internal(obt::Orbit, ::IdDict)
    return Orbit(obt.p, obt.e, obt.i, obt.lan, obt.argpe, obt.epoch, obt.ta)
end

global ktk_orbit_get_p = nothing
global ktk_orbit_get_e = nothing
global ktk_orbit_get_i = nothing
global ktk_orbit_get_lan = nothing
global ktk_orbit_get_argpe = nothing
global ktk_orbit_get_epoch = nothing
global ktk_orbit_get_ta = nothing

global ktk_orbit_set_p = nothing
global ktk_orbit_set_e = nothing
global ktk_orbit_set_i = nothing
global ktk_orbit_set_lan = nothing
global ktk_orbit_set_argpe = nothing
global ktk_orbit_set_epoch = nothing
global ktk_orbit_set_ta = nothing

global ktk_orbit_tof = nothing

global ktk_orbit_new = nothing
global ktk_orbit_free = nothing

function Base.getproperty(obt::Orbit, s::Symbol)
    if getfield(obt, :_inner) == Ptr{Opaque}(0) && s != :_inner
        error("Tried to access field $(s) on an invalidated Orbit")
    end
    return @match s begin
        :p => ccall(ktk_orbit_get_p, Float64, (Ptr{Opaque},), getfield(obt, :_inner))
        :e => ccall(ktk_orbit_get_e, Float64, (Ptr{Opaque},), getfield(obt, :_inner))
        :i => ccall(ktk_orbit_get_i, Float64, (Ptr{Opaque},), getfield(obt, :_inner))
        :lan => ccall(ktk_orbit_get_lan, Float64, (Ptr{Opaque},), getfield(obt, :_inner))
        :argpe =>
            ccall(ktk_orbit_get_argpe, Float64, (Ptr{Opaque},), getfield(obt, :_inner))
        :epoch =>
            UT(ccall(ktk_orbit_get_epoch, Float64, (Ptr{Opaque},), getfield(obt, :_inner)))
        :ta => ccall(ktk_orbit_get_ta, Float64, (Ptr{Opaque},), getfield(obt, :_inner))
        s => getfield(obt, s)
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
        :p => ccall(
            ktk_orbit_set_p,
            Nothing,
            (Ptr{Opaque}, Float64),
            getfield(obt, :_inner),
            convert(Float64, val),
        )
        :e => ccall(
            ktk_orbit_set_e,
            Nothing,
            (Ptr{Opaque}, Float64),
            getfield(obt, :_inner),
            convert(Float64, val),
        )
        :i => ccall(
            ktk_orbit_set_i,
            Nothing,
            (Ptr{Opaque}, Float64),
            getfield(obt, :_inner),
            convert(Float64, val),
        )
        :lan => ccall(
            ktk_orbit_set_lan,
            Nothing,
            (Ptr{Opaque}, Float64),
            getfield(obt, :_inner),
            convert(Float64, val),
        )
        :argpe => ccall(
            ktk_orbit_set_argpe,
            Nothing,
            (Ptr{Opaque}, Float64),
            getfield(obt, :_inner),
            convert(Float64, val),
        )
        :epoch => ccall(
            ktk_orbit_set_epoch,
            Nothing,
            (Ptr{Opaque}, Float64),
            getfield(obt, :_inner),
            convert(Float64, val),
        )
        :ta => ccall(
            ktk_orbit_set_ta,
            Nothing,
            (Ptr{Opaque}, Float64),
            getfield(obt, :_inner),
            convert(Float64, val),
        )
        s => setfield!(obt, s, val)
    end
end

export time_of_flight

"""
    time_of_flight(r0::Float64, r::Float64, ta0::Float64, ta::Float64,
    p::Float64, mu::Float64)

Calculate the time of flight between the provided points on an orbit.

# Arguments

- `r0::Float64`: Radius at initial true anomaly (`km`).
- `r::Float64`: Radius at final true anomaly (`km`).
- `ta0::Float64`: Initial true anomaly (`rad`).
- `ta::Float64`: Final true anomaly (`rad`).
- `p::Float64`: Orbit semi-parameter (`km`).
- `mu::Float64`: The orbited body's standard gravitational parameter
  (`km³/s²`).
"""
function time_of_flight(r0::Float64, r::Float64, ta0::Float64, ta::Float64, p::Float64, mu::Float64)::Float64
    ccall(ktk_orbit_tof, Float64, (Float64, Float64, Float64, Float64, Float64, Float64), r0, r, ta0, ta, p, mu)
end

# TODO: forward these to FFI?

export semimajoraxis, apsis, apoapsis, periapsis, meanmotion, period

"""
    semimajoraxis(obt::Orbit)

Calculate the semi-major axis of the orbit. Note that for parabolic
orbits this is infinite.
"""
semimajoraxis(obt::Orbit) = obt.p / (1.0 - obt.e^2)

"""
    apsis(obt::Orbit, apsis::Apsis)

Calculate the radius at the provided apsis.
"""
function apsis(obt::Orbit, apsis::Apsis)
    @match apsis begin
        $Apoapsis => obt.p / (1.0 - obt.e)
        $Periapsis => obt.p / (1.0 + obt.e)
    end
end

"""
    apoapsis(obt::Orbit)

Calculate the radius at apoapsis.
"""
apoapsis(obt::Orbit) = apsis(obt, Apoapsis)

"""
    periapsis(obt::Orbit)

Calculate the radius at periapsis.
"""
periapsis(obt::Orbit) = apsis(obt, Periapsis)

"""
    meanmotion(obt::Orbit, mu::Float64)

Calculate the mean motion of the orbit. `mu` is the orbited body's
standard gravitational parameter (`km³/s²`).
"""
function meanmotion(obt::Orbit, mu::Float64)
    if abs(obt.e - 1.0) < 1e-6
        # parabolic
        2.0 * sqrt(mu / obt.p^3)
    elseif obt.e < 1.0
        # elliptic
        sqrt(mu / semimajoraxis(obt)^3)
    elseif obt.e > 1.0
        # hyperbolic
        sqrt(mu / (-semimajoraxis(obt))^3)
    else
        error("meanmotion: unknown orbit regime")
    end
end

"""
    period(obt::Orbit, mu::Float64)

Calculate the period of the orbit. `mu` is the orbited body's standard
gravitational parameter (`km³/s²`). Note that for parabolic and
hyperbolic orbits this may not be well defined.
"""
function period(obt::Orbit, mu::Float64)
    2π / meanmotion(obt, mu)
end

end # KerbTk.Orbits
