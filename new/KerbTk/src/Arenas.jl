module Arenas

using MsgPack: MsgPack

export AbstractId, Arena

abstract type AbstractId end

mutable struct Arena{Id<:AbstractId,T} <: AbstractDict{Id,T}
    _inner::Dict{Id,T}
    _next_id::Id
end

MsgPack.msgpack_type(::Type{Arena{Id,T}}) where {Id<:AbstractId,T} = MsgPack.StructType()

function Arena{Id,T}() where {Id<:AbstractId,T}
    return Arena{Id,T}(Dict{Id,T}(), convert(Id, UInt64(0)))
end

Base.pairs(x::Arena{Id,T}) where {Id<:AbstractId,T} = x
Base.keytype(::Type{Arena{Id,T}}) where {Id<:AbstractId,T} = Id
Base.valtype(::Type{Arena{Id,T}}) where {Id<:AbstractId,T} = T
Base.keys(x::Arena{Id,T}) where {Id<:AbstractId,T} = keys(x._inner)
Base.values(x::Arena{Id,T}) where {Id<:AbstractId,T} = values(x._inner)

function Base.iterate(a::Arena{Id,T}) where {Id<:AbstractId,T}
    return iterate(a._inner)
end

function Base.iterate(a::Arena{Id,T}, st) where {Id<:AbstractId,T}
    return iterate(a._inner, st)
end

Base.length(a::Arena) = length(a._inner)
Base.isempty(a::Arena) = isempty(a._inner)

function Base.push!(a::Arena{Id,T}, x::T) where {Id<:AbstractId,T}
    id = a._next_id
    a._next_id = convert(Id, convert(UInt64, a._next_id) + UInt64(1))
    a._inner[id] = x
    return id
end

function Base.push!(a::Arena{Id,T}, x::Tuple{Id,T}) where {Id<:AbstractId,T}
    return insert!(a, x[1], x[2])
end

function Base.insert!(a::Arena{Id,T}, id::Id, x::T) where {Id<:AbstractId,T}
    a._inner[id] = x
    if convert(UInt64, id) >= convert(UInt64, a._next_id)
        a._next_id = convert(Id, convert(UInt64, a._next_id) + UInt64(1))
    end
end

function Base.get(a::Arena{Id,T}, id::Id, default::Any) where {Id<:AbstractId,T}
    return get(a._inner, id, default)
end

function Base.haskey(a::Arena{Id,T}, id::Id) where {Id<:AbstractId,T}
    return haskey(a._inner, id)
end

function Base.setindex!(a::Arena{Id,T}, val::T, id::Id) where {Id<:AbstractId,T}
    return a._inner[id] = val
end

function retain!(f::Function, a::Arena{Id,T}) where {Id<:AbstractId,T}
    for v in a._inner
        if !f(v)
            delete!(a._inner, v[1])
        end
    end
end

end
