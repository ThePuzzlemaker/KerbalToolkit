module StateVectors

using StaticArrays: SVector, SMatrix
using Match: @match
using LinearAlgebra: normalize, norm, cross, dot, transpose
using Optim: Optim, optimize, GoldenSection
using Roots: Roots, find_zero, Brent
using Rotations: AngleAxis

using ..Bodies: Body, SolarSystem
using ..Time: UT
using ..Orbits:
    Orbits,
    Orbit,
    Apsis,
    Apoapsis,
    Periapsis,
    pqw_ijk_matrix,
    time_of_flight,
    mean_motion,
    apsis_radius,
    period

export ReferenceFrame,
    BodyCenteredInertial,
    BodyCenteredBodyFixed,
    StateVector,
    propagate,
    next_soi,
    exit_soi,
    intersect_soi_child,
    reframe,
    latlng

"""
The reference frame with which position and velocity are given.

- `BodyCenteredInertial`
    - X: vernal equinox or equivalent
    - Y: completes the right-handed triad
    - Z: along the rotational axis of the body
- `BodyCenteredBodyFixed`
    - X: along the 0°, 0° surface direction
    - Y: completes the right-handed triad
    - Z: along the rotational axis of the body
"""
@enum ReferenceFrame begin
    BodyCenteredInertial
    BodyCenteredBodyFixed
end

"""
    StateVector

A state vector: a complete description of a spacecraft or body's state
at the given time.

# Fields

- `body::Body`: The orbited celestial body
- `frame::ReferenceFrame`: The state vector's reference frame
- `position::SVector{3, Float64}`: The object's current position
- `velocity::SVector{3, Float64}`: The object's current velocity
- `time::UT`: The state vector's current time
"""
struct StateVector
    body::Body
    frame::ReferenceFrame
    position::SVector{3,Float64}
    velocity::SVector{3,Float64}
    time::UT
end

"""
    Orbit(sv::StateVector; tol::Float64=1e-8)

Convert the provided state vector `sv` into an [`Orbit`](@ref). The
provided tolerance `tol` determines what values
eccentricity/inclination must have relative to 0 to be considered
circular/equatorial respectively.

"""
function Orbits.Orbit(sv::StateVector; tol::Float64=1e-8)
    @assert sv.frame == BodyCenteredInertial

    rv = sv.position
    r = norm(rv)

    vv = sv.velocity
    v = norm(vv)

    hv = cross(rv, vv)
    h = norm(hv)

    nv = cross(SVector{3}(0.0, 0.0, 1.0), hv)

    ev = 1 / sv.body.mu * ((v^2 - sv.body.mu / r) * rv - dot(rv, vv) * vv)
    e = norm(ev)

    p = h^2 / sv.body.mu
    i = acos(hv[3] / h)

    circular = e < tol
    equatorial = abs(i) < tol

    (lan, argpe, ta) = if equatorial && !circular
        (
            0,
            # Longitude of periapsis
            mod(atan(ev[2], ev[1]), 2π),
            atan(dot(hv, cross(ev, rv)) / h, dot(rv, ev)),
        )
    elseif !equatorial && circular
        (
            mod(atan(nv[2], nv[1]), 2π),
            0,
            # Argument of latitude
            atan(dot(rv, cross(hv, nv)) / h, dot(rv, nv)),
        )
    elseif equatorial && circular
        (
            0,
            0,
            # True longitude
            mod(atan(rv[2], rv[1]), 2π),
        )
    else
        a = p / (1 - e^2)

        ta = if a > 0
            e_se = dot(rv, vv) / sqrt(sv.body.mu * a)
            e_ce = r * dot(vv, vv) / sv.body.mu - 1
            2 * atan(sqrt((1 + e) / (1 - e)) * tan(atan(e_se, e_ce) / 2))
        else
            e_sh = dot(rv, vv) / sqrt(-sv.body.mu * a)
            e_ch = r * dot(vv, vv) / sv.body.mu - 1
            F = log((e_ch + e_sh) / (e_ch - e_sh)) / 2
            2 * atan(sqrt((e + 1) / (e - 1)) * tanh(F / 2))
        end
        @info "ta=$ta"

        lan = mod(atan(nv[2], nv[1]), 2π)

        px = dot(rv, nv)
        py = dot(rv, cross(hv, nv)) / h
        argpe = mod(atan(py, px) - ta, 2π)

        (lan, argpe, ta)
    end

    ta = mod(ta + π, 2π) - π
    return Orbit(p, e, i, lan, argpe, ta, sv.time)
end

"""
    StateVector(obt::Orbit, body::Body)

Convert the orbit `obt` to a state vector around the provided `body`.
"""
function StateVector(obt::Orbit, body::Body)
    pqw_r = obt.p / (1 + obt.e * cos(obt.ta))
    pqw_rv = pqw_r * SVector{3}(cos(obt.ta), sin(obt.ta), 0)
    pqw_vv = sqrt(body.mu / obt.p) * SVector{3}(-sin(obt.ta), obt.e + cos(obt.ta), 0)

    mat = pqw_ijk_matrix(obt.i, obt.lan, obt.argpe)

    return StateVector(body, BodyCenteredInertial, mat * pqw_rv, mat * pqw_vv, obt.epoch)
end

function _propagate_inner(sv::StateVector, dt::UT; tol::Float64=1e-7, maxiter::Integer=500)
    return _propagate_inner(sv, dt.value; tol, maxiter)
end

function _propagate_inner(
    sv::StateVector, dt::Float64; tol::Float64=1e-7, maxiter::Integer=500
)
    @assert maxiter > 0
    @assert sv.frame == BodyCenteredInertial

    if abs(dt) < 1e-6
        return sv
    end

    alpha = -norm(sv.velocity)^2 / sv.body.mu + 2 / norm(sv.position)

    xn_new = if alpha > 1e-6
        if abs(alpha - 1) <= tol
            return nothing
        end
        sqrt(sv.body.mu) * dt * alpha
    elseif abs(alpha) <= 1e-6
        h = cross(sv.position, sv.velocity)
        p = norm(h)^2 / sv.body.mu
        s = atan(1.0, 3.0 * dt * sqrt(sv.body.mu / p^3))
        w = atan(cbrt(tan(s)))
        sqrt(p) * 2.0 / tan(2.0 * w)
    elseif alpha <= -1e-6
        a = 1 / alpha
        sign(dt) *
        sqrt(-a) *
        log(
            (-2 * sv.body.mu * alpha * dt) / (
                dot(sv.position, sv.velocity) +
                sign(dt) * sqrt(-sv.body.mu * a) * (1 - norm(sv.position) * alpha)
            ),
        )
    else
        return nothing
    end

    xn = NaN
    c2 = NaN
    c3 = NaN
    r = NaN
    psi = NaN

    dot_r0v0 = dot(sv.position, sv.velocity)
    norm_r0 = norm(sv.position)
    sqrt_mu = sqrt(sv.body.mu)
    iter = 0
    while iter < maxiter
        xn = xn_new
        psi = xn^2 * alpha
        if psi > 1e-6
            c2 = (1.0 - cos(sqrt(psi))) / psi
            c3 = (sqrt(psi) - sin(sqrt(psi))) / (psi * sqrt(psi))
        elseif psi < -1e-6
            c2 = (1 - cosh(sqrt(-psi))) / psi
            c3 = (sinh(sqrt(-psi)) - sqrt(-psi)) / sqrt((-psi)^3)
        else
            c2 = 1 / 2
            c3 = 1 / 6
        end
        r =
            xn * xn * c2 +
            dot_r0v0 / sqrt_mu * xn * (1.0 - psi * c3) +
            norm_r0 * (1.0 - psi * c2)
        xn_new =
            xn +
            (
                sqrt_mu * dt - xn * xn * xn * c3 - dot_r0v0 / sqrt_mu * xn * xn * c2 -
                norm_r0 * xn * (1.0 - psi * c3)
            ) / r

        if abs(xn_new - xn) < tol
            break
        end

        iter += 1
    end
    if iter == maxiter
        return nothing
    end

    xn = xn_new

    f = 1 - xn^2 / norm_r0 * c2
    g = dt - xn^3 / sqrt_mu * c3
    gdot = 1 - xn^2 / r * c2
    fdot = sqrt_mu / (r * norm_r0) * xn * (psi * c3 - 1.0)

    if isnan(f) || isnan(g) || isnan(gdot) || isnan(fdot)
        return nothing
    end

    position = f * sv.position + g * sv.velocity
    velocity = fdot * sv.position + gdot * sv.velocity

    return StateVector(sv.body, sv.frame, position, velocity, sv.time + dt)
end

"""
    propagate(args...; kwargs...)

Propagate an orbit to a specified point

# Arguments
- `sv::StateVector`: The state vector to propagate.
- The second argument differs depending on the method:
    - `dt::Float64`: The delta-T to propagate by
    - `apsis::Apsis`: The apsis to propagate to
- `system::SolarSystem`: The solar system to propagate within

# Keywords
- `tol::Float64=1e-7`: The floating-point tolerance for convergence
  processes
- `maxiter::Float64=30000`: The maximum number of iteration for
  iterative processes
- `soi::Bool=true`: Whether or not to propagate using spheres of
  influence. **It is important that this is set to `false` when
  propagating body orbits, otherwise unexpected failures may occur.**
"""
function propagate end

# function propagate(
#     sv::StateVector,
#     apsis::Apsis,
#     system::SolarSystem;
#     tol::Float64=1e-7,
#     maxiter::Integer=30000,
#     soi::Bool=true,
# )
#     obt = Orbit(sv; tol)

#     if apsis == Apoapsis && obt.e >= 1.0
#         # Apoapsis is not very well-defined for para- and hyper-bolic
#         # orbits
#         return nothing
#     end

#     ta = @match apsis begin
#         $Periapsis => 0.0
#         $Apoapsis => π
#     end
#     tof = time_of_flight(
#         norm(sv.position), apsis_radius(obt, apsis), obt.ta, ta, obt.p, sv.body.mu
#     )
#     tof = mod(tof, period(obt, sv.body.mu))
#     return propagate(sv, tof, system; tol, maxiter, soi)
# end

function propagate(
    sv::StateVector,
    dt::Float64,
    system::SolarSystem;
    tol::Float64=1e-7,
    maxiter::Integer=30000,
    soi::Bool=true,
)
    if soi
        target_tag = sv.time + dt
        soi = next_soi(sv, system; tol, maxiter)
        while !isnothing(soi)
            if soi.time.value < target_tag.value
                dt = target_tag.value - soi.time.value
                sv = soi
            else
                break
            end
            soi = next_soi(sv, system; tol, maxiter)
        end
        _propagate_inner(sv, dt; tol, maxiter)
    else
        _propagate_inner(sv, dt; tol, maxiter)
    end
end

"""
    next_soi(args...; kwargs...)

Calculate the next sphere-of-influence change, and propagate the state
vector to the boundary. Note that the returned state vector is at the
exact (or, as exact as the method allows) time of SOI change, with the
position and velocity in the *next* SOI.

# Arguments
- `sv::StateVector`: The state vector to calculate against
- `system::SolarSystem`: The solar system to calculate within

# Keywords
- `tol::Float64=1e-7`: The floating point tolerance for convergence
  processes
- `maxiter::Integer=30000`: The maximum number of iterations for
  iterative processes
"""
function next_soi(
    sv::StateVector, system::SolarSystem; tol::Float64=1e-7, maxiter::Integer=30000
)
    @assert sv.frame == BodyCenteredInertial

    exit = exit_soi(sv; tol, maxiter)
    if !isnothing(exit)
        return _next_soi_ancestor(exit, system; tol, maxiter)
    else
        return _next_soi_child(sv, system; tol, maxiter)
    end
end

"""
    exit_soi(sv::StateVector; tol::Float64=1e-7, maxiter::Integer=500)

Propagate the provided state vector to the next sphere-of-influence
exit. Note that the returned state vector is at the exact (or, as
exact as the method allows) time of SOI exit, with the position and
velocity in the *current* SOI.

# Arguments
- `sv::StateVector`: The state vector to propagate

# Keywords
- `tol::Float64=1e-7`: The floating point tolerance for convergence
  processes
- `maxiter::Integer=500`: The maximum number of iterations for
  iterative processes
"""
function exit_soi(sv::StateVector; tol::Float64=1e-7, maxiter::Integer=500)
    @assert sv.frame == BodyCenteredInertial

    obt = Orbit(sv; tol)
    alpha = (obt.p - sv.body.soi) / (obt.e * sv.body.soi)

    if abs(alpha) > 1
        return nothing
    end

    ta = acos(alpha)
    tof = time_of_flight(norm(sv.position), sv.body.soi, obt.ta, ta, obt.p, sv.body.mu)

    iter = 0
    while (isnan(tof) || tof < 0) && iter < maxiter
        ta += 2π
        tof = time_of_flight(norm(sv.position), sv.body.soi, obt.ta, ta, obt.p, sv.body.mu)
        iter += 1
    end

    if iter == maxiter
        return nothing
    end

    return _propagate_inner(sv, tof; tol, maxiter)
end

function _next_soi_ancestor(
    sv::StateVector, system::SolarSystem; tol::Float64=1e-7, maxiter::Integer=500
)
    @assert sv.frame == BodyCenteredInertial

    name = sv.body.parent
    isnothing(name) && return nothing

    cur_body = sv.body
    pos = sv.position
    vel = sv.velocity

    body = get(system.bodies, name, nothing)
    while !isnothing(body)
        cur_body_sv_prop = _propagate_inner(
            StateVector(cur_body.ephem, body), sv.time - cur_body.ephem.epoch; tol, maxiter
        )
        pos += cur_body_sv_prop.position
        vel += cur_body_sv_prop.velocity

        if isinf(body.soi) || (norm(pos)^2 - body.soi) >= tol
            return StateVector(body, BodyCenteredInertial, pos, vel, sv.time)
        end

        name = body.parent
        isnothing(name) && return nothing

        cur_body = body
        body = get(system.bodies, name, nothing)
    end

    return nothing
end

function _next_soi_child(
    sv::StateVector, system::SolarSystem; tol::Float64=1e-7, maxiter::Integer=30000
)
    @assert sv.frame == BodyCenteredInertial

    satellites = sv.body.satellites
    satellites = map(satellites) do (body)
        body = get(system.bodies, body, nothing)
        isnothing(body) && return nothing

        body_sv_prop = _propagate_inner(
            StateVector(body.ephem, sv.body), sv.time - body.ephem.epoch; tol, maxiter
        )
        isnothing(body_sv_prop) && return nothing

        res = intersect_soi_child(sv, body_sv_prop, body; tol, maxiter)
        # Try to prevent re-intersecting our previous SOI when there
        # is no re-intersection. This hopefully shouldn't trigger
        # erroneously.
        if !isnothing(res) && abs((res.time - sv.time).value) < 1e-6
            return nothing
        end

        res
    end
    satellites = filter(x -> !isnothing(x), satellites)
    try
        (_, ix) = findmin(satellites) do (x)
            (x.time - sv.time).value
        end
        return satellites[ix]
    catch _
        return nothing
    end
end

"""
    intersect_soi_child(args...; kwargs...)

Calculate the SOI entry state vector from the state vector `sv` and
`child_body` (with state vector `soi_child` where `soi_child.time ==
sv.time`), if any.

# Arguments
- `sv::StateVector`: The state vector to intersect.
- `soi_child::StateVector`: The state vector of `child_body` at
  `sv.time`. Note that `soi_child.time == sv.time` must hold.
- `child_body::Body`: The child body to intersect with. Note that
  `child_body.parent == sv.body.name` must hold.

# Keywords
- `tol::Float64=1e-7`: The floating point tolerance for convergence
  processes
- `maxiter::Integer=30000`: The maximum number of iterations for
  iterative processes
"""
function intersect_soi_child(
    sv::StateVector,
    soi_child::StateVector,
    child_body::Body;
    tol::Float64=1e-7,
    maxiter::Integer=30000,
)
    @assert sv.frame == BodyCenteredInertial
    @assert soi_child.frame == BodyCenteredInertial
    @assert soi_child.time == sv.time
    @assert child_body.parent == sv.body.name

    r_soi = child_body.soi
    sv_s = sv
    sv_c = soi_child

    initial = (dt) -> begin
        sv_s_prop = _propagate_inner(sv_s, dt; tol, maxiter)
        isnothing(sv_s_prop) && return Inf

        sv_c_prop = _propagate_inner(sv_c, dt; tol, maxiter)
        isnothing(sv_c_prop) && return Inf

        abs(norm(sv_s_prop.position - sv_c_prop.position))
    end

    obt = Orbit(sv; tol)
    initial_res = optimize(
        initial,
        0.0,
        2π / mean_motion(obt, sv.body.mu),
        GoldenSection();
        rel_tol=0.0,
        abs_tol=tol,
        iterations=maxiter,
    )
    if isnothing(initial_res) ||
        !Optim.converged(initial_res) ||
        Optim.minimum(initial_res) > r_soi
        return nothing
    end

    r_closest = Optim.minimum(initial_res)
    xn = Optim.minimizer(initial_res)

    if abs(r_closest - r_soi) > tol
        final = (dt) -> begin
            sv_s_prop = _propagate_inner(sv_s, dt; tol, maxiter)
            isnothing(sv_s_prop) && return Inf

            sv_c_prop = _propagate_inner(sv_c, dt; tol, maxiter)
            isnothing(sv_c_prop) && return Inf

            norm(sv_s_prop.position - sv_c_prop.position) - r_soi
        end

        try
            xn = find_zero(final, (0.0, xn), Brent(); atol=tol, rtol=0.0, maxiters=maxiter)
        catch _
            return nothing
        end
    end

    sv_s = _propagate_inner(sv_s, xn; tol, maxiter)
    isnothing(sv_s) && return nothing

    sv_c = _propagate_inner(sv_c, xn; tol, maxiter)
    isnothing(sv_c) && return nothing

    return StateVector(
        child_body,
        BodyCenteredInertial,
        sv_s.position - sv_c.position,
        sv_s.velocity - sv_c.velocity,
        sv.time + xn,
    )
end

function _matrix_bcbf_to_bci(sv::StateVector)
    rotang = mod(
        sv.body.rotini + mod(sv.time.value, sv.body.rotperiod) * norm(sv.body.angvel), 2π
    )
    angvel = normalize(sv.body.angvel)

    return SMatrix{3,3}(AngleAxis(-rotang, angvel.x, angvel.y, angvel.z))
end

function _bci_bcbf(sv::StateVector)
    rotation_bci_to_bcbf = inv(_matrix_bcbf_to_bci(sv))

    position = rotation_bci_to_bcbf * sv.position

    rot_deriv = norm(sv.body.angvel)

    rotang = mod(sv.body.rotini + mod(sv.time.value, sv.body.rotperiod) * rot_deriv, 2π)
    deriv_matrix = inv(
        SMatrix{3}(
            -rot_deriv * sin(rotang),
            -rot_deriv * cos(rotang),
            0.0,
            rot_deriv * cos(rotang),
            -rot_deriv * sin(rotang),
            0.0,
            0.0,
            0.0,
            0.0,
        ),
    )

    velocity = rotation_bci_to_bcbf * sv.velocity + deriv_matrix * position

    return StateVector(sv.body, BodyCenteredBodyFixed, position, velocity, sv.time)
end

function _bcbf_bci(sv::StateVector)
    rotation_bcbf_to_bci = _matrix_bcbf_to_bci(sv)

    position = rotation_bcbf_to_bci * sv.position

    rot_deriv = norm(sv.body.angvel)

    rotang = mod(sv.body.rotini + mod(sv.time.value, sv.body.rotperiod) * rot_deriv, 2π)
    deriv_matrix = SMatrix{3}(
        -rot_deriv * sin(rotang),
        -rot_deriv * cos(rotang),
        0.0,
        rot_deriv * cos(rotang),
        -rot_deriv * sin(rotang),
        0.0,
        0.0,
        0.0,
        0.0,
    )

    velocity = rotation_bcbf_to_bci * sv.velocity + deriv_matrix * position

    return StateVector(sv.body, BodyCenteredInertial, position, velocity, sv.time)
end

"""
    reframe(sv::StateVector, frame::ReferenceFrame)

Convert the state vector `sv` into the provided `frame`.
"""
function reframe(sv::StateVector, frame::ReferenceFrame)
    return @match (sv.frame, frame) begin
        ($BodyCenteredInertial, $BodyCenteredBodyFixed) => _bci_bcbf(sv)
        ($BodyCenteredBodyFixed, $BodyCenteredInertial) => _bcbf_bci(sv)
        (frame, frame) => sv
        frame => error("Unknown reference frame(s)")
    end
end

"""
    latlng(sv::StateVector)

Return the `(latitude, longitude)` of the provided state vector `sv`.
"""
function latlng(sv::StateVector)
    sv = reframe(sv, BodyCenteredBodyFixed)
    pos = normalize(sv.position)
    lng = atan(pos.y, pos.x)
    lat = asin(pos.z)
    return (lat, lng)
end

end
