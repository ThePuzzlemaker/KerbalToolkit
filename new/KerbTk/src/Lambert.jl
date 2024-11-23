"""
A relatively quick and robust Lambert's problem solver.

This solver is based on ["Revisiting Lambert's Problem" (Izzo
2014)][1] and [poliastro's `poliastro.iod.izzo`][2].  (the latter of
which is licensed under MIT).

[1]: https://arxiv.org/abs/1403.2705
[2]: https://github.com/poliastro/poliastro/blob/c7d12e9b715d3fd60f2be233af707d5b97617d39/src/poliastro/iod/izzo.py
"""
module Lambert

using StaticArrays: SVector
using LinearAlgebra: norm, cross, normalize

using ..Math: hyp2f1

export lambert

function calc_y(x::Float64, lambda::Float64)
    return sqrt((1 - lambda^2 * (1 - x^2)))
end

function calc_psi(x::Float64, y::Float64, lambda::Float64)
    if -1.0 <= x <= 1.0 # elliptic
        acos(x * y + lambda * (1 - x^2))
    elseif x > 1.0 # hyperbolic
        asinh((y - x * lambda) - sqrt(x^2 - 1))
    else # parabolic
        0.0
    end
end

function tof(x::Float64, y::Float64, t0::Float64, lambda::Float64, m::Integer)
    t = if m == 0 && 0.6 <= x <= 1.4
        eta = y - lambda * x
        s1 = 1 / 2 * (1 - lambda - x * eta)
        q = 4 / 3 * hyp2f1(s1)
        1 / 2 * (eta^3 * q + 4 * lambda * eta)
    else
        1 / (1 - x^2) * ((calc_psi(x, y, lambda) + m) / sqrt(abs(1 - x^2)) - x + lambda * y)
    end

    return t - t0
end

function dtof(x::Float64, y::Float64, t::Float64, lambda::Float64)
    return (3 * t * x - 2 + 2 * lambda^3 * x / y) / (1 - x^2)
end

function d2tof(x::Float64, y::Float64, t::Float64, dt::Float64, lambda::Float64)
    return (3 * t + 5 * x * dt + 2 * (1 - lambda^2) * lambda^3 / y^3) / (1 - x^2)
end

function d3tof(
    x::Float64, y::Float64, ::Float64, dt::Float64, ddt::Float64, lambda::Float64
)
    return (7 * x * ddt + 8 * dt - 6 * (1 - lambda^2) * lambda^5 * x / y^5) / (1 - x^2)
end

function calc_tmin(lambda::Float64, m::Integer, tol::Float64, maxiter::Integer)
    if abs(lambda - 1.0) < tol
        return tof(0.0, calc_y(0.0, lambda), 0.0, lambda, m)
    elseif m == 0
        return 0.0
    else
        # set x_i > 0 to avoid FP issues at λ = -1
        x_i = 0.1
        t_i = tof(x_i, calc_y(x_i, lambda), 0.0, lambda, m)
        x_tmin = halley(x_i, t_i, lambda, m, tol, maxiter)
        isnothing(x_tmin) && return nothing
        return tof(x_tmin, calc_y(x_tmin, lambda), 0.0, lambda, m)
    end
end

function halley(
    x0::Float64, t0::Float64, lambda::Float64, m::Integer, tol::Float64, maxiter::Integer
)
    iter = maxiter
    while iter > 0
        y = calc_y(x0, lambda)
        f = tof(x0, y, t0, lambda, m)
        t = f + t0
        df = dtof(x0, y, t, lambda)
        d2f = d2tof(x0, y, t, df, lambda)
        if d2f == 0.0
            return nothing
        end
        d3f = d3tof(x0, y, t, df, d2f, lambda)

        x = x0 - 2 * df * d2f / (2 * d2f^2 - df * d3f)

        if abs(x - x0) < tol
            return x
        end

        x0 = x
        iter -= 1
    end

    return nothing
end

function householder(
    x0::Float64, t0::Float64, lambda::Float64, m::Integer, tol::Float64, maxiter::Integer
)
    iter = maxiter
    while iter > 0
        y = calc_y(x0, lambda)
        f = tof(x0, y, t0, lambda, m)
        t = f + t0
        df = dtof(x0, y, t, lambda)
        d2f = d2tof(x0, y, t, df, lambda)
        d3f = d3tof(x0, y, t, df, d2f, lambda)

        x = x0 - f * ((df^2 - f * d2f / 2) / (df * (df^2 - f * d2f) + d3f * f^2 / 6))

        if abs(x - x0) < tol
            return x
        end

        x0 = x
        iter -= 1
    end

    return nothing
end

function findxy(lambda::Float64, t::Float64, tol::Float64, maxiter::Integer)
    if abs(lambda) >= 1.0 || t <= 0.0
        return nothing
    end

    mmax = convert(Int32, floor(t / π))
    t00 = acos(lambda) + lambda * sqrt(1 - lambda^2)
    if t < t00 + mmax * π && mmax > 0
        tmin = calc_tmin(lambda, mmax, tol, maxiter)
        isnothing(tmin) && return nothing

        if t < tmin
            mmax -= 1
        end
    end

    t1 = 2 / 3 * (1 - lambda^3)
    x0 = if t >= t00
        (t00 / t)^(2 / 3) - 1
    elseif t < t1
        5 / 2 * t1 / t * (t1 - t) / (1 - lambda^5) + 1
    else
        exp(log(2) * log(t / t00) / log(t1 / t00)) - 1
    end

    xs = []
    ys = []
    x = householder(x0, t, lambda, 0, tol, maxiter)
    isnothing(x) && return nothing
    y = calc_y(x, lambda)
    push!(xs, x)
    push!(ys, y)

    while mmax > 0
        m = mmax
        x0l = (((m * π + π) / 8 * t)^(2 / 3) - 1) / (((m * π + π) / 8 * t)^(2 / 3) + 1)
        x0r = ((8 * t / m * π)^(2 / 3) - 1) / ((8 * t / m * π)^(2 / 3) + 1)

        xl = householder(x0l, t, lambda, m, tol, maxiter)
        if !isnothing(xl)
            yl = calc_y(xl, lambda)
            push!(xs, xl)
            push!(ys, yl)
        end

        xr = householder(x0r, t, lambda, m, tol, maxiter)
        if !isnothing(xr)
            yr = calc_y(xr, lambda)
            push!(xs, xr)
            push!(ys, yr)
        end

        mmax -= 1
    end

    return (xs, ys)
end

"""
    lambert(args...; kwargs...)

Lambert's problem.

Given position vectors `r1v` and `r2v`, time-of-flight `tof`, and
gravitational parameter `mu`, calculate all possible velocity vectors
`v1` and `v2` for the corresponding orbits.

Note that this function is unit-agnostic, however the distance units
of the position vectors `r1`, `r2` and the time unit of the
time-of-flight `tof` must match with the respective distance and time
units of the gravitational parameter `mu`.

# Arguments
- `r1v::SVector{3, Float64}`, `r2v::SVector{3, Float64}`: position
  vectors.
- `tof::Float64`: time-of-flight.
- `mu::Float64`: standard gravitational parameter for the orbited
  body.

# Keywords
- `tol::Float64`: floating point tolerance. Values smaller than
  `1e-15` will typically fail to converge.
- `maxiter::Integer`: maximum iterations for convergence. This
  function will panic if it could not converge within `maxiter`
  iterations.
"""
function lambert(
    r1v::SVector{3,Float64},
    r2v::SVector{3,Float64},
    tof::Float64,
    mu::Float64;
    tol::Float64=1e-15,
    maxiter::Int64=500,
)
    r1 = norm(r1v)
    r2 = norm(r2v)
    cv = r2v - r1v
    c = norm(cv)

    s = 1 / 2 * (r1 + r2 + c)
    ir1 = r1v / r1
    ir2 = r2v / r2
    ih = cross(ir1, ir2)
    ih = normalize(ih)

    lambda = sqrt(1 - c / s)
    (it1, it2) = if (r1v[1] * r2v[2] - r1v[2] * r2v[1]) < 0.0
        lambda = -lambda
        (cross(ir1, ih), cross(ir2, ih))
    else
        (cross(ih, ir1), cross(ih, ir2))
    end

    t = tof * sqrt(2mu / s^3)
    res = findxy(lambda, t, tol, maxiter)
    isnothing(res) && return []
    (xs, ys) = res

    ret = []
    gamma = sqrt(mu * s / 2)
    rho = (r1 - r2) / c
    sigma = sqrt(1 - rho^2)
    for (x, y) in zip(xs, ys)
        vr1 = gamma * ((lambda * y - x) - rho * (lambda * y + x)) / r1
        vr2 = -gamma * ((lambda * y - x) + rho * (lambda * y + x)) / r2
        vt1 = gamma * sigma * (y + lambda * x) / r1
        vt2 = gamma * sigma * (y + lambda * x) / r2
        v1 = vr1 * (r1v / r1) + vt1 * it1
        v2 = vr2 * (r2v / r2) + vt2 * it2
        push!(ret, (v1, v2))
    end

    return ret
end

end
