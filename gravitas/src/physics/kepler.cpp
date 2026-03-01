#include <gravitas/physics/kepler.h>
#include <gravitas/numerics.h>

using namespace gravitas::physics;

double KeplerOrbit::apsisRadius(const Apsis apsis) const {
  switch (apsis) {
    case APOAPSIS:
      return apoapsisRadius();
    case PERIAPSIS:
      return periapsisRadius();
    default:
      assert(false);
  }
}

double KeplerOrbit::meanMotion(const double mu) const {
  if (std::abs(e - 1.0) < 1e-6) {
    // parabolic
    return 2.0 * std::sqrt(mu / (p * p * p));
  } else if (e < 1.0) {
    // elliptic
    const auto a = semimajorAxis();
    return std::sqrt(mu / (a * a * a));
  } else if (e > 1.0) {
    // hyperbolic
    const auto a = semimajorAxis();
    return std::sqrt(-mu / (a * a * a));
  } else {
    assert(false);
  }
}

std::pair<Vector3d, Vector3d> KeplerOrbit::rvPqw(const KeplerBody& body) const {
  const auto r = p / (1.0 + e * std::cos(ta));
  const auto rv = Vector3d(r * std::cos(ta), r * std::sin(ta), 0.0);
  const auto vv =
      std::sqrt(body.mu / p) * Vector3d(-std::sin(ta), e + std::cos(ta), 0.0);
  return std::make_pair(rv, vv);
}

Matrix3d KeplerOrbit::pqwIjkMatrix(const double i,
                                   const double lan,
                                   const double argpe) {
  const auto m11 = std::cos(lan) * std::cos(argpe) -
                   std::sin(lan) * std::sin(argpe) * std::cos(i);
  const auto m12 = -std::cos(lan) * std::sin(argpe) -
                   std::sin(lan) * std::cos(argpe) * std::cos(i);
  const auto m13 = std::sin(lan) * std::sin(i);
  const auto m21 = std::sin(lan) * std::cos(argpe) +
                   std::cos(lan) * std::sin(argpe) * std::cos(i);
  const auto m22 = -std::sin(lan) * std::sin(argpe) +
                   std::cos(lan) * std::cos(argpe) * std::cos(i);
  const auto m23 = -std::cos(lan) * std::sin(i);
  const auto m31 = std::sin(argpe) * std::sin(i);
  const auto m32 = std::cos(argpe) * std::sin(i);
  const auto m33 = std::cos(i);

  Matrix3d m;
  m << m11, m12, m13, m21, m22, m23, m31, m32, m33;
  return m;
}

StateVector KeplerOrbit::stateBci(
    const std::shared_ptr<KeplerBody>& body) const {
  auto [rv, vv] = rvPqw(*body);
  const auto mat = pqwIjkMatrix();
  rv = mat * rv;
  vv = mat * vv;
  return StateVector{.position = rv,
                     .velocity = vv,
                     .parent = std::make_optional(body),
                     .time = epoch};
}

static double e_to_ta(const double e, const double ecc) {
  return 2.0 *
         std::atan(std::sqrt((1.0 + ecc) / (1.0 - ecc)) * std::tan(e / 2.0));
}

static double f_to_ta(const double f, const double ecc) {
  return 2.0 *
         std::atan(std::sqrt((ecc + 1.0) / (ecc - 1.0)) * std::tanh(f / 2.0));
}

KeplerOrbit::KeplerOrbit(const StateVector& sv, const double tol) {
  const auto body = sv.expectParent();

  const auto rv = sv.position;
  const auto r = rv.norm();
  const auto vv = sv.velocity;
  const auto v = vv.norm();
  const auto hv = rv.cross(vv);
  const auto h = hv.norm();
  const auto nv = Vector3d(0, 0, 1).cross(hv);
  const auto ev =
      1.0 / body->mu * ((v * v - body->mu / r) * rv - rv.dot(vv) * vv);
  p = (h * h) / body->mu;
  e = ev.norm();
  i = std::acos(hv[2] / h);

  const auto circular = e < tol;
  const auto equatorial = std::abs(i) < tol;

  if (equatorial && !circular) {
    lan = 0.0;
    // Longitude of periapsis
    argpe =
        numerics::modEuclid(std::atan2(ev[1], ev[0]), 2.0 * std::numbers::pi);
    ta = std::atan2(hv.dot(ev.cross(rv)), rv.dot(ev));
  } else if (!equatorial && circular) {
    lan = numerics::modEuclid(std::atan2(nv[1], nv[0]), 2.0 * std::numbers::pi);
    argpe = 0.0;
    // Argument of latitude
    ta = std::atan2(rv.dot(hv.cross(nv)) / h, rv.dot(nv));
  } else if (equatorial) {
    lan = 0.0;
    argpe = 0.0;
    // True longitude
    ta = numerics::modEuclid(std::atan2(rv[1], rv[0]), 2.0 * std::numbers::pi);
  } else {
    const auto a = p / (1.0 - e * e);
    const auto mua = body->mu * a;

    if (a > 0.0) {
      const auto e_se = rv.dot(vv) / std::sqrt(mua);
      const auto e_ce = r * vv.dot(vv) / body->mu - 1.0;
      ta = e_to_ta(std::atan2(e_se, e_ce), e);
    } else {
      const auto e_sh = rv.dot(vv) / std::sqrt(-mua);
      const auto e_ch = r * vv.squaredNorm() / body->mu - 1.0;
      ta = f_to_ta(std::log((e_ch + e_sh) / (e_ch - e_sh)) / 2.0, e);
    }

    lan = numerics::modEuclid(std::atan2(nv[1], nv[0]), 2.0 * std::numbers::pi);
    const auto px = rv.dot(nv);
    const auto py = rv.dot(hv.cross(nv)) / h;
    argpe =
        numerics::modEuclid(std::atan2(py, px) - ta, 2.0 * std::numbers::pi);
  }

  ta = numerics::modEuclid(ta + std::numbers::pi, 2.0 * std::numbers::pi) -
       std::numbers::pi;
  epoch = sv.time;
}

static void calcC2C3(const double psi, double& c2, double& c3) {
  if (psi > 1e-6) {
    c2 = (1.0 - std::cos(std::sqrt(psi))) / psi;
    c3 = (std::sqrt(psi) - std::sin(std::sqrt(psi))) / (psi * std::sqrt(psi));
  } else if (psi < -1e-6) {
    c2 = (1.0 - std::cosh(std::sqrt(-psi))) / psi;
    c3 = (std::sinh(std::sqrt(-psi)) - std::sqrt(-psi)) /
         std::sqrt(-psi * psi * psi);
  } else {
    c2 = 1.0 / 2.0;
    c3 = 1.0 / 6.0;
  }
}

std::optional<StateVector> gravitas::physics::keplerPropagate(
    const StateVector& sv,
    const double dt,
    const double tol,
    const unsigned int maxIter) {
  const auto body = sv.expectParent();
  const auto alpha =
      -sv.velocity.squaredNorm() / body->mu + 2.0 / sv.position.norm();

  double xnNew;
  if (alpha > 1e-6) {
    if (std::abs(alpha - 1.0) <= tol)
      return {};

    xnNew = std::sqrt(body->mu) * dt * alpha;
  } else if (std::abs(alpha) <= 1e-6) {
    const auto h = sv.position.cross(sv.velocity);
    const auto p = h.squaredNorm() / body->mu;
    const auto s = std::atan2(1.0, 3.0 * dt * std::sqrt(body->mu / (p * p * p)));
    const auto w = std::atan(std::cbrt(std::tan(s)));
    xnNew = std::sqrt(p) * 2.0 * 1.0 / std::tan(2.0 * w);
  } else if (alpha <= -1e-6) {
    const auto a = 1.0 / alpha;
    const auto dtSign = (dt >= 0.0 ? 1.0 : -1.0);
    xnNew = dtSign * std::sqrt(-a) *
            std::log((-2.0 * body->mu * alpha * dt) /
                     (sv.position.dot(sv.velocity) +
                      dtSign * std::sqrt(-body->mu * a) *
                          (1.0 - sv.position.norm() * alpha)));
  } else {
    return {};
  }

  double xn;
  double c2 = std::nan("");
  double c3 = std::nan("");
  double r = std::nan("");
  double psi = std::nan("");
  const auto dotR0V0 = sv.position.dot(sv.velocity);
  const auto normR0 = sv.position.norm();
  const auto sqrtMu = std::sqrt(body->mu);
  auto iter = 0;
  while (iter < maxIter) {
    xn = xnNew;
    psi = xn * xn * alpha;
    calcC2C3(psi, c2, c3);
    r = xn * xn * c2 + dotR0V0 / sqrtMu * xn * (1.0 - psi * c3) +
        normR0 * (1.0 - psi * c2);
    xnNew = xn +
            (sqrtMu * dt - xn * xn * xn * c3 - dotR0V0 / sqrtMu * xn * xn * c2 -
             normR0 * xn * (1.0 - psi * c3)) /
                r;

    if (std::abs(xnNew - xn) < tol) {
      break;
    }

    iter++;
  }
  if (iter == maxIter) {
    return {};
  }

  xn = xnNew;

  const auto f = 1.0 - xn * xn / normR0 * c2;
  const auto g = dt - xn * xn * xn / sqrtMu * c3;

  const auto gdot = 1.0 - xn * xn / r * c2;
  const auto fdot = sqrtMu / (r * normR0) * xn * (psi * c3 - 1.0);

  auto svNew = sv;
  svNew.position = f * sv.position + g * sv.velocity;
  svNew.velocity = fdot * sv.position + gdot * sv.velocity;

  return std::move(svNew);
}

StateVector KeplerBody::propagate(const double dt) const {
  return keplerPropagate(
      ephem.stateBci(std::static_pointer_cast<KeplerBody>(expectParent())), dt).value();
}

StateVector KeplerVessel::propagate(const double dt) const {
  return keplerPropagate(stateVector(), dt).value();
}
