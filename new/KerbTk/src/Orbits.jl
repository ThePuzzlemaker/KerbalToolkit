module Orbits

using MsgPack: MsgPack

using Match: @match
using StaticArrays: SMatrix

using ..Time: UT

export Apsis,
    Apoapsis,
    Periapsis,
    OrbitalNode,
    Ascending,
    Descending,
    Orbit,
    apsis_radius,
    periapsis_radius,
    apoapsis_radius,
    semimajor_axis,
    mean_motion,
    period,
    pqw_ijk_matrix

@enum Apsis begin
    Apoapsis
    Periapsis
end

@enum OrbitalNode begin
    Ascending
    Descending
end

"""
    Orbit

An orbit, as described by the 6 Keplerian elements and an epoch time.

# Fields
- `p::Float64`: Semi-latus rectum (`km`)
- `e::Float64`: Eccentricity (dimensionless)
- `i::Float64`: Inclination (`rad`)
- `lan::Float64`: Longitude of ascending node (`rad`)
- `argpe::Float64`: Argument of periapsis (`rad`)
- `ta::Float64`: True anomaly (`rad`)
- `epoch::UT`: Epoch at the current true anomaly
"""
struct Orbit
    p::Float64
    e::Float64
    i::Float64
    lan::Float64
    argpe::Float64
    ta::Float64
    epoch::UT
end

MsgPack.msgpack_type(::Type{Orbit}) = MsgPack.StructType()

"""
    apsis_radius(obt::Orbit, apsis::Apsis)

Calculate the radius at the provided apsis.
"""
function apsis_radius(obt::Orbit, apsis::Apsis)
    return @match apsis begin
        $Apoapsis => obt.p / (1 - obt.e)
        $Periapsis => obt.p / (1 + obt.e)
    end
end

"""
    periapsis_radius(obt::Orbit)

Calculate the radius at periapsis.
"""
periapsis_radius(obt::Orbit) = apsis_radius(obt, Periapsis)

"""
    apoapsis_radius(obt::Orbit)

Calculate the radius at apoapsis.
"""
apoapsis_radius(obt::Orbit) = apsis_radius(obt, Apoapsis)

"""
    semimajor_axis(obt::Orbit)

Calculate the semimajor axis of the orbit. Note that for parabolic
orbits this is infinite.
"""
semimajor_axis(obt::Orbit) = obt.p / (1 - obt.e^2)

"""
    mean_motion(obt::Orbit, mu::Float64)

Calculate the mean motion of the orbit `obt` with the given standard
gravitational parameter `mu`.
"""
function mean_motion(obt::Orbit, mu::Float64)
    if isapprox(obt.e, 1; atol=1e-6) # parabolic
        2 * sqrt(mu / obt.p^3)
    elseif obt.e < 1 # elliptic
        sqrt(mu / semimajor_axis(obt)^3)
    elseif obt.e > 1 # hyperbolic
        sqrt(-mu / semimajor_axis(obt)^3)
    else
        error("Unknown eccentricity regime")
    end
end

"""
    period(obt::Orbit, mu::Float64)

Calculate the period of the orbit `obt` with the given standard
gravitational parameter `mu`. Note that this is not well-defined
for parabolic and hyperbolic orbits.
"""
function period(obt::Orbit, mu::Float64)
    return 2π / mean_motion(obt, mu)
end

"""
    pqw_ijk_matrix(args...)

Calculate the matrix which transforms perifocal (PQW) coordinates to
IJK (body-centered inertial) coordinates.

# Arguments
- `i::Float64`: Inclination (`rad`)
- `lan::Float64`: Longitude of ascending node (`rad`)
- `argpe::Float64`: Argument of periapsis (`rad`)
"""
function pqw_ijk_matrix(i::Float64, lan::Float64, argpe::Float64)
    m11 = cos(lan) * cos(argpe) - sin(lan) * sin(argpe) * cos(i)
    m12 = -cos(lan) * sin(argpe) - sin(lan) * cos(argpe) * cos(i)
    m13 = sin(lan) * sin(i)

    m21 = sin(lan) * cos(argpe) + cos(lan) * sin(argpe) * cos(i)
    m22 = -sin(lan) * sin(argpe) + cos(lan) * cos(argpe) * cos(i)
    m23 = -cos(lan) * sin(i)

    m31 = sin(argpe) * sin(i)
    m32 = cos(argpe) * sin(i)
    m33 = cos(i)

    return SMatrix{3,3}(m11, m12, m13, m21, m22, m23, m31, m32, m33)
end

"""
    time_of_flight(args...)

Calculate the time of flight between the provided points on an orbit.

# Arguments

- `r0::Float64`: Radius at initial true anomaly (`km`).
- `r::Float64`: Radius at final true anomaly (`km`).
- `ta0::Float64`: Initial true anomaly (`rad`).
- `ta::Float64`: Final true anomaly (`rad`).
- `p::Float64`: Orbit semi-parameter (`km`).
- `mu::Float64`: The orbited body's standard gravitational parameter
  (`km³/s²`).
"""
function time_of_flight(
    r0::Float64, r::Float64, ta0::Float64, ta::Float64, p::Float64, mu::Float64
)
    delta_ta = ta - ta0
    cos_delta_ta = cos(delta_ta)
    k = r0 * r * (1 - cos_delta_ta)
    ell = r0 + r
    m = r0 * r * (1 + cos_delta_ta)
    alpha = (2m - ell^2) * p^2 + 2k * ell * p - k^2
    a = (m * k * p) / alpha
    f = 1 - r / p * (1 - cos_delta_ta)
    g = (r0 * r * sin(delta_ta)) / sqrt(mu * p)
    f_dot = sqrt(mu / p) * tan(delta_ta / 2) * ((1 - cos_delta_ta) / p - 1 / r0 - 1 / r)

    if abs(alpha) < 1e-6 # Parabolic
        c = sqrt(r0^2 + r^2 - 2r0 * r * cos_delta_ta)
        s = (r0 + r + c) / 2
        2 / 3 * sqrt(s^3 / 2mu * (1 - ((s - c) / s)^(3 / 2)))
    elseif abs(a) < 1e-6 || a > 0 # Elliptical
        cos_delta_E = 1 - (r0 / a) * (1 - f)
        sin_delta_E = (-r0 * r * f_dot) / sqrt(mu * a)
        delta_E = atan(sin_delta_E, cos_delta_E)
        g + sqrt(a^3 / mu) * (delta_E - sin_delta_E)
    elseif a < 0 # Hyperbolic
        cosh_delta_H = 1 + (f - 1) * (r0 / a)
        delta_H = acosh(cosh_delta_H)
        g + sqrt((-a)^3 / mu) * (sinh(delta_H) - delta_H)
    else
        error("Unknown orbit regime")
    end
end

end
