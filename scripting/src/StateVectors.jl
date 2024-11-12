module StateVectors

using Match
using StaticArrays

using ..Bodies: Body, Bodies
using ..Orbits: Orbits, Orbit
using ..KerbTk: UT
import ..Missions

export ReferenceFrame, BodyCenteredInertial, BodyCenteredBodyFixed, Opaque, StateVector, sv_bci

@enum ReferenceFrame begin
    BodyCenteredInertial
    BodyCenteredBodyFixed
end

abstract type Opaque end

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

function sv_bci(obt::Orbit, body::Body)
    getfield(obt, :_inner) == Ptr{Orbits.Opaque}(0) &&
        error("Tried to access an invalidated Orbit")
    getfield(body, :_inner) == Ptr{Bodies.Opaque}(0) &&
        error("Tried to access an invalidated Body")
    StateVector(ccall(ktk_orbit_sv_bci, Ptr{Opaque}, (Ptr{Orbits.Opaque}, Ptr{Bodies.Opaque}), obt._inner, body._inner), nothing, false)
end

function propagate(sv::StateVector, dt::Float64; tol::Float64=1e-7, maxiter::Int=30000, soi::Bool=true)
    res = if soi
        ccall(ktk_sv_propagate, Ptr{Opaque}, (Ptr{Missions.Opaque}, Ptr{Opaque}, Float64, Float64, UInt64), Missions.ktk_mission_ptr, sv._inner, dt, tol, convert(UInt64, maxiter))
    else
        ccall(ktk_sv_propagate_no_soi, Ptr{Opaque}, (Ptr{Opaque}, Float64, Float64, UInt64), sv._inner, dt, tol, convert(UInt64, maxiter))
    end
    if res == Ptr{Opaque}(0)
        error("Failed to propagate state vector")
    end
    StateVector(res, nothing, false)
end

function Orbits.Orbit(sv::StateVector)
    convert(Orbit, sv)
end

function Base.convert(::Type{Orbit}, sv::StateVector)
    getfield(sv, :_inner) == Ptr{Opaque}(0) &&
        error("Tried to access an invalidated StateVector")
    Orbit(ccall(ktk_sv_to_orbit, Ptr{Orbits.Opaque}, (Ptr{Opaque},), sv._inner), nothing, false)
end

function Base.deepcopy(sv::StateVector)
    StateVector(sv.body, sv.frame, sv.position, sv.velocity, sv.time)
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

end # KerbTk.StateVectors
