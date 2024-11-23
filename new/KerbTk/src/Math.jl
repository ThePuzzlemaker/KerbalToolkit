module Math

using MsgPack: MsgPack

export hyp2f1, H1, evaluate, h1_add!

using ..Misc: SortedList, find_first_position

"""
    hyp2f1(x::Float64)

The hypergeometric function `₂F₁(3, 1, 5/2, x)`
"""
function hyp2f1(x::Float64)
    if x >= 1.0
        Inf
    else
        res = 1.0
        term = 1.0
        i = 0.0
        while true
            term = term * (3 + i) * (1 + i) / (5 / 2 + i) * x / (i + 1)
            res_old = res
            res += term
            if abs(res_old - res) < 1e-8
                return res
            end
            i += 1
        end
    end
end

mutable struct HFrame
    in_tangent::Float64
    out_tangent::Float64
    time::Float64
    value::Float64
    auto_tangent::Bool
end

MsgPack.msgpack_type(::Type{HFrame}) = MsgPack.StructType()

mutable struct H1
    min_time::Float64
    max_time::Float64
    last_lo::Int64
    list::SortedList{Float64,HFrame}
end

H1() = H1(floatmax(Float64), floatmin(Float64), -1, SortedList{Float64,HFrame}())

MsgPack.msgpack_type(::Type{H1}) = MsgPack.StructType()

function interpolant(
    x1::Float64,
    y1::Float64,
    yp1::Float64,
    x2::Float64,
    y2::Float64,
    yp2::Float64,
    x::Float64,
)
    t = (x - x1) / (x2 - x1)
    h00 = 2t^3 - 3t^2 + 1
    h10 = t^3 - 2t^2 + t
    h01 = -2t^3 + 3t^2
    h11 = t^3 - t^2
    return h00 * y1 + h10 * (x2 - x1) * yp1 + h01 * y2 + h11 * (x2 - x1) * yp2
end

function h1_add!(h::H1, time::Float64, value::Float64)
    insert!(h.list, time, HFrame(0.0, 0.0, time, value, true))
    h.min_time = min(h.min_time, time)
    h.max_time = max(h.max_time, time)
    recompute_tangents(h, convert(Int64, find_first_position(h.list, time)[2]))
    h.last_lo = -1
    return nothing
end

function h1_add!(
    h::H1, time::Float64, value::Float64, in_tangent::Float64, out_tangent::Float64
)
    insert!(h.list, time, HFrame(in_tangent, out_tangent, time, value, false))
    h.min_time = min(h.min_time, time)
    h.max_time = max(h.max_time, time)
    recompute_tangents(h, convert(Int64, find_first_position(h.list, time)[2]))
    h.last_lo = -1
    return nothing
end

function h1_add!(h::H1, time::Float64, value::Float64, tangent::Float64)
    (res, at) = find_first_position(h.list, time)
    if res
        h.list.values[at].value = value
        h.list.values[at].out_tangent = tangent
    else
        h1_add!(h, time, value, tangent, tangent)
    end
    return nothing
end

function recompute_tangents(h::H1, i::Int64)
    if length(h.list) == 1
        if h.list.values[1].auto_tangent
            h.list.values[1].in_tangent = 0.0
            h.list.values[1].out_tangent = 0.0
        end
        return nothing
    end

    fix_tangent(h, i)

    if i != 1
        fix_tangent(h, i - 1)
    end
    if i != length(h.list)
        fix_tangent(h, i + 1)
    end
end

function fix_tangent(h::H1, i::Int64)
    if !h.list.values[i].auto_tangent
        return nothing
    end

    slope1 = 0.0

    if i < length(h.list)
        right = h.list.values[i + 1]
        slope1 = right.value - h.list.values[i].value
        slope1 /= right.time - h.list.values[i].time

        if i == 1
            h.list.values[i].in_tangent = slope1
            h.list.values[i].out_tangent = slope1
            return nothing
        end
    end

    slope2 = 0.0

    if i > 1
        left = h.list.values[i - 1]
        slope2 = h.list.values[i].value - left.value
        slope2 /= h.list.values[i].time - left.time

        if i == length(h.list)
            h.list.values[i].in_tangent = slope2
            h.list.values[i].out_tangent = slope2
            return nothing
        end
    end

    slope1 += slope2
    slope1 /= 2.0
    h.list.values[i].in_tangent = slope1
    return h.list.values[i].in_tangent = slope2
end

function evaluate(h::H1, t::Float64)
    if isempty(h.list)
        return 0.0
    end

    if t <= h.min_time
        return h.list.values[1].value
    end

    if t >= h.max_time
        return h.list.values[end].value
    end

    hi = find_index(h, t)

    if hi >= 0
        return h.list.values[hi + 1].value
    end

    hi = ~hi

    test_keyframe = h.list.values[hi]
    test_keyframe2 = h.list.values[hi + 1]

    return interpolant(
        test_keyframe.time,
        test_keyframe.value,
        test_keyframe.out_tangent,
        test_keyframe2.time,
        test_keyframe2.value,
        test_keyframe2.in_tangent,
        t,
    )
end

function find_index(h::H1, value::Float64)
    @assert length(h.list) > 1
    @assert value > h.min_time
    @assert value < h.max_time

    if h.last_lo > 0 && value > h.list.keys[h.last_lo + 1]
        if value < h.list.keys[h.last_lo + 2]
            return ~(h.last_lo + 1)
        end

        if value == h.list.keys[h.last_lo + 2]
            h.last_lo += 1
            return h.last_lo
        end

        if value > h.list.keys[h.last_lo + 2] && value < h.list.keys[h.last_lo + 3]
            h.last_lo += 1
            return ~(h.last_lo + 1)
        end
    end

    lo = 0
    hi = length(h.list) - 1

    while lo <= hi
        i = lo + ((hi - lo) >> 1)
        x = h.list.keys[i + 1]
        if value < x
            hi = i - 1
        elseif value == x
            h.last_lo = i
            return i
        else
            lo = i + 1
        end
    end

    h.last_lo = lo - 1

    return ~lo
end

end
