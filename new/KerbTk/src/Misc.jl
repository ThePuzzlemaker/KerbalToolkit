module Misc

using MsgPack: MsgPack

# I agree, this is janky.

mutable struct SortedList{K,V}
    keys::Vector{K}
    values::Vector{V}
end

SortedList{K,V}() where {K,V} = SortedList{K,V}([], [])

MsgPack.msgpack_type(::Type{SortedList{K,V}}) where {K,V} = MsgPack.StructType()

Base.length(l::SortedList) = length(l.keys)
Base.isempty(l::SortedList) = isempty(l.keys)

function binary_search(v::Vector{T}, x::T) where {T}
    max = length(v) + 1
    if max == 1
        return (false, 1)
    end
    base = 1

    while max > 2
        half = div(max, 2)
        mid = base + half
        base = v[mid] > x ? base : mid
        max -= half
    end

    if v[base] == x
        return (true, base)
    elseif v[base] < x
        return (false, base + 1)
    else
        return (false, base)
    end
end

function Base.insert!(l::SortedList{K,V}, key::K, value::V) where {K,V}
    (res, at) = binary_search(l.keys, key)
    if res
        insertion_position = find_insertion_position(l, at, key, value)

        if !isnothing(insertion_position)
            insert!(insertion_position, key, value, l.keys, l.values)
            return nothing
        else
            return nothing
        end
    else
        insert!(l.keys, at, key)
        insert!(l.values, at, value)
        return nothing
    end
end

function find_first_value_of(l::SortedList{K,V}, key::K) where {K,V}
    (res, at) = find_first_position(l, key)
    if res
        l.values[at]
    else
        return nothing
    end
end

function find_first_position(l::SortedList{K,V}, key::K) where {K,V}
    (res, at) = binary_search(l.keys, key)
    if res
        while at > 1 && key == l.keys[at]
            at -= 1
        end
        if at == 1
            if key == l.keys[1]
                return (true, 1)
            else
                return (true, 2)
            end
        else
            return (true, at + 1)
        end
    else
        return (false, at)
    end
end

function find_insertion_position(
    l::SortedList{K,V}, from::Integer, key::K, value::V
) where {K,V}
    keys = l.keys[from:end]
    values = l.values[from:end]

    index = from

    while true
        index += 1

        if !isempty(keys) && !isempty(values)
            other_key = keys[1]
            other_value = values[1]
            if key == other_key
                if value == other_value
                    return nothing
                end
            else
                return (:before, index)
            end
        elseif isempty(keys) && isempty(values)
            return (:last, 0)
        else
            error("unreachable")
        end
        keys = keys[2:end]
        values = values[2:end]
    end
end

function Base.insert!(
    pos::Tuple{Symbol,Integer}, key::K, value::V, keys::Vector{K}, values::Vector{V}
) where {K,V}
    if pos[1] == :before
        insert!(keys, pos[2] - 1, key)
        insert!(values, pos[2] - 1, value)
    elseif pos[1] == :last
        push!(keys, key)
        push!(values, value)
    else
        error("unreachable")
    end
end

end
