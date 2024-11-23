module Bodies

using StaticArrays: SVector

using ..Orbits: Orbit

using MsgPack: MsgPack

"""
    Body

A celestial body.

# Fields
- `mu::Float64`: Standard gravitational parameter (`km^3/s^2`).
- `radius::Float64`: Mean radius of the body's sphere (`km`).
- `ephem::Orbit`: Ephemerides at the starting epoch.
- `rotperiod::Float64`: Rotational period; length of sidereal day
  (`sec`).
- `rotini::Float64`: Initial rotation about the body's spin axis at
  UT=0 (`rad`).
- `satellites::Vector{String}`: Names of bodies orbiting this body.
- `parent::Union{String, Nothing}`: Name of the parent of this body,
  if any.
- `name::String`: Name of this body as displayed in KSP.
- `is_star::Bool`: Is this a star?
- `soi::Float64`: Radius of this body's sphere of influence (`km`).
- `angvel::SVector{3, Float64}`: Angular momentum direction in BCI
  coordinates.
"""
struct Body
    mu::Float64
    radius::Float64
    ephem::Orbit
    rotperiod::Float64
    rotini::Float64
    satellites::Vector{String}
    parent::Union{String,Nothing}
    name::String
    is_star::Bool
    soi::Float64
    angvel::SVector{3,Float64}
end

MsgPack.msgpack_type(::Type{Body}) = MsgPack.StructType()

"""
    SolarSystem

A solar system.

# Fields
- `bodies::Dict{String, Body}`
"""
struct SolarSystem
    bodies::Dict{String,Body}
end

MsgPack.msgpack_type(::Type{SolarSystem}) = MsgPack.StructType()

end
