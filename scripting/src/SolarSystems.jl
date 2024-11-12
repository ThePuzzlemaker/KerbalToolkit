module SolarSystems

export Opaque, SolarSystem

using ..Bodies: Bodies, Body

abstract type Opaque end

mutable struct SolarSystem <: Base.AbstractDict{String,Body}
    _inner::Ptr{Opaque}
    _parent::Any
    function SolarSystem(_inner::Ptr{Opaque}, _parent::Any)
        x = new(_inner, _parent)
        function f(t)
            if getfield(t, :_inner) == Ptr{Opaque}(0)
                error("Tried to destroy an invalidated SolarSystem")
            else
                ccall(ktk_systems_free, Nothing, (Ptr{Opaque},), t._inner)
                t._inner = Ptr{Opaque}(0)
            end
        end
        finalizer(f, x)
    end
end

function Base.iterate(sys::SolarSystem)
    getfield(sys, :_inner) == Ptr{Opaque}(0) &&
        error("Tried to access an invalidated SolarSystem")

    vec = convert(Vector{Tuple{String,Body}}, sys)
    val = get(vec, 1, nothing)
    if isnothing(val)
        nothing
    else
        (val, (vec, 2))
    end
end

function Base.iterate(sys::SolarSystem, (vec, ix))
    getfield(sys, :_inner) == Ptr{Opaque}(0) &&
        error("Tried to access an invalidated SolarSystem")

    val = get(vec, ix, nothing)
    if isnothing(val)
        nothing
    else
        (val, (vec, ix + 1))
    end
end

function Base.length(sys::SolarSystem)
    getfield(sys, :_inner) == Ptr{Opaque}(0) &&
        error("Tried to access an invalidated SolarSystem")

    ccall(ktk_systems_get_size, UInt64, (Ptr{Opaque},), sys._inner)
end

function Base.haskey(sys::SolarSystem, name::String)
    getfield(sys, :_inner) == Ptr{Opaque}(0) &&
        error("Tried to access an invalidated SolarSystem")

    ccall(ktk_systems_has_name, Bool, (Ptr{Opaque}, String), sys._inner, name)
end

function Base.convert(::Type{Vector{Tuple{String,Body}}}, sys::SolarSystem)
    getfield(sys, :_inner) == Ptr{Opaque}(0) &&
        error("Tried to access an invalidated SolarSystem")

    vec = ccall(ktk_systems_to_vec, Any, (Ptr{Opaque},), sys._inner)
    f((key, value)) = (key, Body(value))
    vec = map(f, vec)
end

function Base.get(sys::SolarSystem, name::String, default::Any)
    getfield(sys, :_inner) == Ptr{Opaque}(0) &&
        error("Tried to access an invalidated SolarSystem")

    if haskey(sys, name)
        ptr = ccall(
            ktk_systems_get,
            Ptr{Bodies.Opaque},
            (Ptr{Opaque}, String),
            sys._inner,
            name,
        )
        Body(ptr)
    else
        default
    end
end

Base.valtype(::Type{SolarSystem}) = Body
Base.keytype(::Type{SolarSystem}) = String
Base.pairs(sys::SolarSystem) = sys
function Base.keys(sys::SolarSystem)
    getfield(sys, :_inner) == Ptr{Opaque}(0) &&
        error("Tried to access an invalidated SolarSystem")

    ccall(ktk_systems_get_names, Any, (Ptr{Opaque},), sys._inner)
end
function Base.values(sys::SolarSystem)
    getfield(sys, :_inner) == Ptr{Opaque}(0) &&
        error("Tried to access an invalidated SolarSystem")

    map(Body, ccall(ktk_systems_get_bodies, Any, (Ptr{Opaque},), sys._inner))
end

global ktk_systems_get_names = nothing
global ktk_systems_get_bodies = nothing
global ktk_systems_has_name = nothing
global ktk_systems_get = nothing
global ktk_systems_to_vec = nothing
global ktk_systems_get_size = nothing
global ktk_systems_free = nothing

function Base.show(io::IO, sys::SolarSystem)
    getfield(sys, :_inner) == Ptr{Opaque}(0) &&
        error("Tried to access an invalidated SolarSystem")

    vec = convert(Vector{Tuple{String,Body}}, sys)
    write(io, "SolarSystem(")
    join(io, map(kv -> "$(repr(kv[1])) => Body(…)", vec), ", ")
    write(io, ")")
end

Base.deepcopy_internal(::SolarSystem) = error("Not implemented")

end # KerbTk.SolarSystem
