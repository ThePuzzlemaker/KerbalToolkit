module Encode

using ProtoBuf: ProtoBuf, ProtoDecoder, ProtoEncoder, encode, decode
using ProtoBuf.Codecs: _encode

using ..Schema: Schema

export encode_value, decode_value

function ProtoBuf.encode(x::T) where {T}
    buf = IOBuffer()
    encode(ProtoEncoder(buf), x)
    return take!(buf)
end

function ProtoBuf.decode(buf::Vector{UInt8}, t::Type{T})::T where {T}
    buf = IOBuffer(buf)
    return decode(ProtoDecoder(buf), t)
end

function decode_value(buf::Vector{UInt8}, ::Type{Dict{K,V}}) where {K,V}
    map = Dict{K,V}()
    raw = decode(buf, Schema.Dictionary)
    for entry in raw.entries
        key = decode_value(entry.key, K)
        value = decode_value(entry.value, V)
        map[key] = value
    end
    return map
end

function encode_value(dict::Dict{K,V}) where {K,V}
    entries = []
    for (key, value) in dict
        key = encode_value(key)
        value = encode_value(value)
        push!(entries, Schema.DictionaryEntry(key, value))
    end
    return encode(Schema.Dictionary(entries))
end

function decode_value(buf::Vector{UInt8}, ::Type{Vector{T}}) where {T}
    v = []
    raw = decode(buf, Schema.List)
    for item in raw.items
        push!(v, decode_value(item, T))
    end
    return v
end

function encode_value(v::Vector{T}) where {T}
    items = []
    for item in v
        push!(items, encode_value(item))
    end
    return encode(Schema.List(items))
end

function decode_value(buf::Vector{UInt8}, ::Type{Tuple{T,U,V}}) where {T,U,V}
    tuple = decode(buf, Schema.Tuple)
    return (
        decode_value(tuple.items[1], T),
        decode_value(tuple.items[2], U),
        decode_value(tuple.items[3], V),
    )
end

function encode_value(x::Tuple{T,U,V}) where {T,U,V}
    v = []
    push!(v, encode_value(x[1]))
    push!(v, encode_value(x[2]))
    push!(v, encode_value(x[3]))
    return encode(Schema.Tuple(v))
end

function decode_value(buf::Vector{UInt8}, ::Type{Tuple{T,U,V,W}}) where {T,U,V,W}
    tuple = decode(buf, Schema.Tuple)
    return (
        decode_value(tuple.items[1], T),
        decode_value(tuple.items[2], U),
        decode_value(tuple.items[3], V),
        decode_value(tuple.items[4], W),
    )
end

function encode_value(x::Tuple{T,U,V,W}) where {T,U,V,W}
    v = []
    push!(v, encode_value(x[1]))
    push!(v, encode_value(x[2]))
    push!(v, encode_value(x[3]))
    push!(v, encode_value(x[4]))
    return encode(Schema.Tuple(v))
end

function decode_value(buf::Vector{UInt8}, ::Type{Tuple{T,U,V,W,X}}) where {T,U,V,W,X}
    tuple = decode(buf, Schema.Tuple)
    return (
        decode_value(tuple.items[1], T),
        decode_value(tuple.items[2], U),
        decode_value(tuple.items[3], V),
        decode_value(tuple.items[4], W),
        decode_value(tuple.items[5], X),
    )
end

function encode_value(x::Tuple{T,U,V,W,X}) where {T,U,V,W,X}
    v = []
    push!(v, encode_value(x[1]))
    push!(v, encode_value(x[2]))
    push!(v, encode_value(x[3]))
    push!(v, encode_value(x[4]))
    push!(v, encode_value(x[5]))
    return encode(Schema.Tuple(v))
end

decode_value(::Vector{UInt8}, ::Type{Tuple{}}) = ()
encode_value(::Type{Tuple{}}) = []

function decode_value(buf::Vector{UInt8}, ::Type{String})
    buf = IOBuffer(buf)
    len = decode(ProtoDecoder(buf), UInt64)
    pos = position(buf) + 1
    buf = take!(buf)[pos:end]
    @assert length(buf) == len
    return String(buf)
end

function encode_value(s::String)
    v = IOBuffer()
    _encode(v, UInt64(length(v)))
    write(v, s)
    return take!(v)
end

function decode_value(buf::Vector{UInt8}, ::Type{Union{T,Nothing}}) where {T}
    if buf == [0x0]
        return nothing
    else
        return decode_value(buf, T)
    end
end

function encode_value(x::Union{T,Nothing}) where {T}
    if isnothing(x)
        return [0x0]
    else
        return encode_value(convert(T, x))
    end
end

function decode_value(
    buf::Vector{UInt8}, ::Type{T}
) where {T<:Union{Float32,Float64,UInt32,UInt64,Int32}}
    return decode(ProtoDecoder(IOBuffer(buf)), T)
end

function encode_value(x::T) where {T<:Union{Float32,Float64,UInt32,UInt64,Int32}}
    buf = IOBuffer()
    _encode(buf, x)
    return take!(buf)
end

function decode_value(buf::Vector{UInt8}, ::Type{Bool})
    return decode(ProtoDecoder(IOBuffer(buf)), Int32) != 0
end

function encode_value(x::Bool)
    buf = IOBuffer()
    _encode(buf, Int32(x ? 1 : 0))
    return take!(buf)
end

function decode_value(buf::Vector{UInt8}, ::Type{Vector{UInt8}})
    return decode(ProtoDecoder(IOBuffer(buf)), Vector{UInt8})
end

function encode_value(x::Vector{UInt8})
    buf = IOBuffer()
    encode(ProtoEncoder(buf), x)
    return take!(buf)
end

function decode_value(buf::Vector{UInt8}, ::Type{Schema.Status})
    return decode(ProtoDecoder(IOBuffer(buf)), Schema.Status)
end

using ..KRPC: CelestialBody, AbstractKRPCValue

function encode_value(x::T) where {T<:AbstractKRPCValue}
    buf = IOBuffer()
    _encode(buf, UInt64(x.inner))
    return take!(buf)
end

function decode_value(buf::Vector{UInt8}, ::Type{T}) where {T<:AbstractKRPCValue}
    return T(decode(ProtoDecoder(IOBuffer(buf)), UInt64))
end

end
