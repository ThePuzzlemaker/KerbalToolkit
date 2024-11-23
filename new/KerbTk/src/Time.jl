module Time

using MsgPack: MsgPack

"""
    AbstractDuration

An abstract type representing a duration of time.
"""
abstract type AbstractDuration end

"""
    Duration

A duration of time, represented as a 64-bit floating point number.
"""
struct Duration <: AbstractDuration
    value::Float64
    Duration(sec::Number) = new(Float64(sec))
    function Duration(;
        days::Number, hours::Number, minutes::Number, seconds::Number, millis::Number=0
    )
        return new(
            days * 24 * 60 * 60 + hours * 60 * 60 + minutes * 60 + seconds + millis / 1000
        )
    end
end

"""
    UT

Universal time, the core time reference of Kerbal Space Program.
"""
struct UT <: AbstractDuration
    value::Float64
    UT(sec::Number) = new(Float64(sec))
    function UT(;
        days::Number, hours::Number, minutes::Number, seconds::Number, millis::Number=0
    )
        return new(
            days * 24 * 60 * 60 + hours * 60 * 60 + minutes * 60 + seconds + millis / 1000
        )
    end
end

"""
    GET

Ground-elaped time, equal to universal time minus the mission base time.
"""
struct GET <: AbstractDuration
    value::Float64
    GET(sec::Number) = new(Float64(sec))
    function GET(;
        days::Number, hours::Number, minutes::Number, seconds::Number, millis::Number=0
    )
        return new(
            days * 24 * 60 * 60 + hours * 60 * 60 + minutes * 60 + seconds + millis / 1000
        )
    end
end

Base.:+(t1::UT, t2::Float64) = UT(t1.value + t2)
Base.:+(t1::GET, t2::Float64) = GET(t1.value + t2)
Base.:+(t1::GET, t2::UT) = UT(t1.value + t2.value)
Base.:+(t1::UT, t2::UT) = UT(t1.value + t2.value)
Base.:+(t1::GET, t2::GET) = GET(t1.value + t2.value)
Base.:+(t1::UT, t2::GET) = UT(t1.value + t2.value)

Base.:-(t1::UT, t2::Float64) = UT(t1.value - t2)
Base.:-(t1::GET, t2::Float64) = GET(t1.value - t2)
Base.:-(t1::GET, t2::UT) = UT(t1.value - t2.value)
Base.:-(t1::UT, t2::UT) = UT(t1.value - t2.value)
Base.:-(t1::GET, t2::GET) = GET(t1.value - t2.value)
Base.:-(t1::UT, t2::GET) = UT(t1.value - t2.value)

Base.convert(::Type{Float64}, d::AbstractDuration) = d.value

MsgPack.msgpack_type(::Type{<:AbstractDuration}) = MsgPack.FloatType()
function MsgPack.to_msgpack(::MsgPack.FloatType, x::T) where {T<:AbstractDuration}
    return convert(Float64, x)
end
MsgPack.from_msgpack(::Type{T}, x::Float64) where {T<:AbstractDuration} = T(x)

"""
    millis(d::AbstractDuration)

Get the number of milliseconds within the duration
"""
millis(d::AbstractDuration) = round(d.value * 1000) % 1000

"""
    seconds(d::AbstractDuration)

Get the number of seconds within the duration
"""
seconds(d::AbstractDuration) = floor(d.value) % 60

"""
    minutes(d::AbstractDuration)

Get the number of minutes within the duration
"""
minutes(d::AbstractDuration) = floor(d.value / 60) % 60

"""
    hours(d::AbstractDuration)

Get the number of hours within the duration
"""
hours(d::AbstractDuration) = floor(d.value / (60 * 60)) % 24

"""
    days(d::AbstractDuration)

Get the number of days within the duration
"""
days(d::AbstractDuration) = floor(d.value / (60 * 60 * 24))

"""
    whole_hours(d::AbstractDuration)

Get the number of whole hours within the duration. This function
differs from [`hours`](@ref) as it returns an unbounded value, i.e.,
calling `whole_hours` on an `AbstractDuration` of 2 days would return
48 hours.
"""
whole_hours(d::AbstractDuration) = floor(d.value / (60 * 60))

end
