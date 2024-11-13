"""
State vectors and associated transformations and calculations.
"""
module StateVectors

using Match
using StaticArrays
using LinearAlgebra

using ..Bodies: Body, Bodies
using ..Orbits: Orbits, Orbit
using ..KerbTk: UT
using ..Enums: Apsis
using ..Support: @check

export ReferenceFrame, BodyCenteredInertial, BodyCenteredBodyFixed, Opaque, StateVector, propagate, reframe, frenet

"""
The reference frame with which position and velocity are given.

- `BodyCenteredInertial`
    - X: vernal equinox or equivalent
    - Y: completes the right-handed triad
    - Z: along the rotational axis of the body
- `BodyCenteredBodyFixed`
    - X: along the 0°, 0° surface direction
    - Y: completes the right-handed triad
    - Z: along the rotational axis of the body
"""
@enum ReferenceFrame begin
    BodyCenteredInertial
    BodyCenteredBodyFixed
end

abstract type Opaque end

"""
A state vector, a complete description of a spacecraft or body's state
at the given time.

# Fields

- `body::Body`: The orbited celestial body
- `frame::ReferenceFrame`: The state vector's reference frame
- `position::SVector{3, Float64}`: The object's current position
- `velocity::SVector{3, Float64}`: The object's current velocity
- `time::UT`: The state vector's current time
"""
mutable struct StateVector
    _inner::Ptr{Opaque}
    _parent::Any
    _readonly::Bool
    function StateVector(
        body::Body,
        frame::ReferenceFrame,
        position::SVector{3,Float64},
        velocity::SVector{3,Float64},
        time::UT,
    )
        _inner = ccall(
            ktk_sv_new,
            Ptr{Opaque},
            (
                Ptr{Bodies.Opaque},
                UInt8,
                Tuple{Float64,Float64,Float64},
                Tuple{Float64,Float64,Float64},
                Float64,
            ),
            body._inner,
            convert(UInt8, Int(frame)),
            (position[1], position[2], position[3]),
            (velocity[1], velocity[2], velocity[3]),
            time.inner,
        )
        x = new(_inner, nothing, false)
        function f(t)
            if getfield(t, :_inner) == Ptr{Opaque}(0)
                error("Tried to destroy an invalidated StateVector")
            else
                ccall(ktk_sv_free, Nothing, (Ptr{Opaque},), t._inner)
                t._inner = Ptr{Opaque}(0)
            end
        end
        finalizer(f, x)
    end

    function StateVector(_inner::Ptr{Opaque}, _parent::Any, _readonly::Bool)
        new(_inner, _parent, _readonly)
    end
end

function Base.show(io::IO, sv::StateVector)
    write(io, "StateVector($(repr(sv.body)), $(repr(sv.frame)), $(repr(sv.position)), $(repr(sv.velocity)), $(repr(sv.time)))")
end

global ktk_sv_new = nothing
global ktk_sv_free = nothing

global ktk_sv_get_body = nothing
global ktk_sv_get_frame = nothing
global ktk_sv_get_position = nothing
global ktk_sv_get_velocity = nothing
global ktk_sv_get_time = nothing

global ktk_sv_set_body = nothing
global ktk_sv_set_frame = nothing
global ktk_sv_set_position = nothing
global ktk_sv_set_velocity = nothing
global ktk_sv_set_time = nothing

global ktk_orbit_sv_bci = nothing
global ktk_sv_to_orbit = nothing
global ktk_sv_propagate = nothing
global ktk_sv_propagate_no_soi = nothing
global ktk_sv_reframe = nothing
global ktk_sv_propagate_to_apsis = nothing

function StateVector(obt::Orbit, body::Body; frame::ReferenceFrame=BodyCenteredInertial)
    getfield(obt, :_inner) == Ptr{Orbits.Opaque}(0) &&
        error("Tried to access an invalidated Orbit")
    getfield(body, :_inner) == Ptr{Bodies.Opaque}(0) &&
        error("Tried to access an invalidated Body")
    frame != BodyCenteredInertial && error("Only Orbit to BCI StateVector conversions are currently implemented")
    StateVector(ccall(ktk_orbit_sv_bci, Ptr{Opaque}, (Ptr{Orbits.Opaque}, Ptr{Bodies.Opaque}), obt._inner, body._inner), nothing, false)
end

"""
    propagate(sv::StateVector, dt::Float64; tol::Float64=1e-7,
    maxiter::Int=30000, soi::Bool=true)

Propagate the provided state vector `sv` by `dt` seconds.

# Arguments

- `sv::StateVector`: The state vector to propagate.
- `dt::Float64`: The amount of time by which to propagate.
- `tol::Float64`: Floating-point tolerance.
- `maxiter::Int`: Maximum number of iterations.
- `soi::Bool`: Whether or not to propagate across spheres of
  influence. Note: this should be `false` when propagating orbits of
  celestial bodies to prevent issues.
"""
function propagate(sv::StateVector, dt::Float64; tol::Float64=1e-7, maxiter::Int=30000, soi::Bool=true)
    getfield(sv, :_inner) == Ptr{Opaque}(0) && error("Tried to access an invlaidated StateVector")
    res = if soi
        ccall(ktk_sv_propagate, Ptr{Opaque}, (Ptr{Opaque}, Float64, Float64, UInt64), sv._inner, dt, tol, convert(UInt64, maxiter))
    else
        ccall(ktk_sv_propagate_no_soi, Ptr{Opaque}, (Ptr{Opaque}, Float64, Float64, UInt64), sv._inner, dt, tol, convert(UInt64, maxiter))
    end
    if res == Ptr{Opaque}(0)
        error("Failed to propagate state vector")
    end
    StateVector(res, nothing, false)
end

"""
    propagate(sv::StateVector, apsis::Apsis; tol::Float64=1e-7,
    maxiter::Int=30000, soi::Bool=true)

Propagate the provided state vector `sv` to the provided `apsis`.

# Arguments

- `sv::StateVector`: The state vector to propagate.
- `apsis::Apsis`: The apsis to propagate towards.
- `tol::Float64`: Floating-point tolerance.
- `maxiter::Int`: Maximum number of iterations.
- `soi::Bool`: Whether or not to propagate across spheres of
  influence. Note: this should be `false` when propagating orbits of
  celestial bodies to prevent issues.
"""
function propagate(sv::StateVector, apsis::Apsis; tol::Float64=1e-7, maxiter::Int=30000, soi::Bool=true)
    getfield(sv, :_inner) == Ptr{Opaque}(0) && error("Tried to access an invlaidated StateVector")
    res = ccall(ktk_sv_propagate_to_apsis, Ptr{Opaque}, (Ptr{Opaque}, UInt8, Float64, UInt64, Bool), sv._inner, convert(UInt8, Int(apsis)), tol, convert(UInt64, maxiter), soi)
    if res == Ptr{Opaque}(0)
        error("Failed to propagate state vector")
    end
    StateVector(res, nothing, false)
end

"""
    reframe(sv::StateVector, frame::ReferenceFrame)

Translate between two compatible reference frames. When `frame` is
equal to `sv`'s current frame, this acts as a no-op.
"""
function reframe(sv::StateVector, frame::ReferenceFrame)
    getfield(sv, :_inner) == Ptr{Opaque}(0) && error("Tried to access an invalidated StateVector")
    StateVector(ccall(ktk_sv_reframe, Ptr{Opaque}, (Ptr{Opaque}, UInt8), getfield(sv, :_inner), convert(UInt8, Int(frame))), nothing, false)
end

"""
    Orbit(sv::StateVector)

Convert a StateVector into an orbit. Note that due to struct
dependencies, this loses some information (specifically, the orbited
body).
"""
function Orbits.Orbit(sv::StateVector)
    getfield(sv, :_inner) == Ptr{Opaque}(0) &&
        error("Tried to access an invalidated StateVector")
    Orbit(ccall(ktk_sv_to_orbit, Ptr{Orbits.Opaque}, (Ptr{Opaque},), sv._inner), nothing, false)
end

function Base.deepcopy_internal(sv::StateVector, dict::IdDict)
    StateVector(Base.deepcopy_internal(sv.body, dict), sv.frame, Base.deepcopy_internal(sv.position, dict), Base.deepcopy_internal(sv.velocity, dict), sv.time)
end

function Base.getproperty(sv::StateVector, s::Symbol)
    if getfield(sv, :_inner) == Ptr{Opaque}(0) && s != :_inner
        error("Tried to access field $(s) on an invalidated StateVector")
    end
    return @match s begin
        :body => Body(
            ccall(
                ktk_sv_get_body,
                Ptr{Bodies.Opaque},
                (Ptr{Opaque},),
                getfield(sv, :_inner),
            ),
        )
        :frame => ReferenceFrame(
            ccall(ktk_sv_get_frame, UInt8, (Ptr{Opaque},), getfield(sv, :_inner)),
        )
        :position =>
            ccall(ktk_sv_get_position, Any, (Ptr{Opaque},), getfield(sv, :_inner))           
        :velocity => 
            ccall(ktk_sv_get_velocity, Any, (Ptr{Opaque},), getfield(sv, :_inner))
        :time => UT(ccall(ktk_sv_get_time, Float64, (Ptr{Opaque},), getfield(sv, :_inner)))
        s => getfield(sv, s)
    end
end

function Base.setproperty!(sv::StateVector, s::Symbol, val::Any)
    if getfield(sv, :_inner) == Ptr{Opaque}(0) && s != :_inner
        error("Tried to set field $(s) on an invalidated StateVector")
    end
    if getfield(sv, :_readonly)
        error("Tried to set field $(s) on a readonly StateVector")
    end
    @match s begin
        :body => ccall(
            ktk_sv_set_body,
            Nothing,
            (Ptr{Opaque}, Ptr{Bodies.Opaque}),
            getfield(sv, :_inner),
            val._inner,
        )
        :frame => ccall(
            ktk_sv_set_frame,
            Nothing,
            (Ptr{Opaque}, UInt8),
            getfield(sv, :_inner),
            convert(UInt8, Int(val)),
        )
        :position => ccall(
            ktk_sv_set_position,
            Nothing,
            (Ptr{Opaque}, Tuple{Float64,Float64,Float64}),
            getfield(sv, :_inner),
            (convert(Float64, val[1]), convert(Float64, val[2]), convert(Float64, val[3])),
        )
        :velocity => ccall(
            ktk_sv_set_velocity,
            Nothing,
            (Ptr{Opaque}, Tuple{Float64,Float64,Float64}),
            getfield(sv, :_inner),
            (convert(Float64, val[1]), convert(Float64, val[2]), convert(Float64, val[3])),
        )
        :time => ccall(
            ktk_sv_set_time,
            Nothing,
            (Ptr{Opaque}, Float64),
            getfield(sv, :_inner),
            convert(Float64, val),
        )
        s => setfield!(sv, s, val)
    end
end

"""
    time_of_flight(sv::StateVector, ta::Float64)

Calculate the time of flight between the current state vector `sv`'s
position and the desired true anomaly `ta`.
"""
function Orbits.time_of_flight(sv::StateVector, ta::Float64)
    obt = Orbit(sv)
    ta0 = obt.ta
    r0 = obt.p / (1.0 + obt.e * cos(ta0))
    r = obt.p / (1.0 + obt.e * cos(ta))
    Orbits.time_of_flight(r0, r, ta0, ta, obt.p, sv.body.mu)
end

# TODO: forward this to FFI?
"""
    frenet(sv::StateVector)

Calculate the matrix which converts the Frenet frame (i.e.,
`SVector{3, Float64}(prograde, normal, radial)`) into the `sv`'s
reference frame at the given time.

To convert the other way around, use this matrix's inverse.
"""
function frenet(sv::StateVector)
    t = normalize(sv.velocity)
    n = normalize(cross(sv.position, sv.velocity))
    b = cross(t, n)
    SMatrix{3,3}(hcat(t, n, b))
end

end # KerbTk.StateVectors
