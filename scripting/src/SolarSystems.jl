module SolarSystems

export Opaque, SolarSystem

using ..Bodies: Bodies, Body

abstract type Opaque end

mutable struct SolarSystem <: Base.AbstractDict{String, Body}
    _inner::Ptr{Opaque}
    _parent::Any
end

function Base.iterate(sys::SolarSystem)
    vec = convert(Vector{Tuple{String, Body}}, sys)
    val = get(vec, 1, nothing)
    if isnothing(val)
        nothing
    else
        (val, (vec, 2))
    end
end

function Base.iterate(::SolarSystem, (vec, ix))
    val = get(vec, ix, nothing)
    if isnothing(val)
        nothing
    else
        (val, (vec, ix+1))
    end
end

function Base.length(sys::SolarSystem)
    ccall(ktk_systems_get_size, UInt64, (Ptr{Opaque},), sys._inner)
end

function Base.haskey(sys::SolarSystem, name::String)
    ccall(ktk_systems_has_name, Bool, (Ptr{Opaque}, String), sys._inner, name)
end

function Base.convert(::Type{Vector{Tuple{String, Body}}}, sys::SolarSystem)
    vec = ccall(ktk_systems_to_vec, Any, (Ptr{Opaque},), sys._inner)
    f((key, value)) = (key, Body(value))
    vec = map(f, vec)
end

function Base.get(sys::SolarSystem, name::String, default::Any)
    if haskey(sys, name)
        ptr = ccall(ktk_systems_get, Ptr{Bodies.Opaque}, (Ptr{Opaque}, String), sys._inner, name)
        Body(ptr)
    else
        default
    end
end

Base.valtype(::Type{SolarSystem}) = Body
Base.keytype(::Type{SolarSystem}) = String
Base.pairs(sys::SolarSystem) = sys
Base.keys(sys::SolarSystem) = ccall(ktk_systems_get_names, Any, (Ptr{Opaque},), sys._inner)
function Base.values(sys::SolarSystem)
    map(Body, ccall(ktk_systems_get_bodies, Any, (Ptr{Opaque},), sys._inner))
end

global ktk_systems_get_names  = nothing
global ktk_systems_get_bodies = nothing
global ktk_systems_has_name   = nothing
global ktk_systems_get        = nothing
global ktk_systems_to_vec     = nothing
global ktk_systems_get_size   = nothing

function Base.show(io::IO, system::SolarSystem)
    vec = convert(Vector{Tuple{String, Body}}, system)
    write(io, "SolarSystem(")
    join(io, map(kv -> "$(repr(kv[1])) => Body(…)", vec), ", ")
    write(io, ")")
end

Base.deepcopy_internal(::SolarSystem) = error("Not implemented")

end # KerbTk.SolarSystem
