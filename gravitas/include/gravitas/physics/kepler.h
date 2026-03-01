#pragma once

#include <memory>
#include <numbers>
#include <optional>

#include "rigidbody.h"

namespace gravitas::physics {
enum Apsis { APOAPSIS, PERIAPSIS };

class KeplerBody;

struct KeplerOrbit final {
  /// Semi-latis rectum (km).
  double p = 0.0;
  /// Eccentricity (dimensionless).
  double e = 0.0;
  /// Inclination (radians).
  double i = 0.0;
  /// Longitude of ascending node (radians).
  double lan = 0.0;
  /// Argument of periapsis (radians).
  double argpe = 0.0;
  /// The epoch at true anomaly (UT seconds).
  double epoch = 0.0;
  /// True anomaly (radians).
  double ta = 0.0;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(KeplerOrbit, p, e, i, lan, argpe, epoch, ta);

  KeplerOrbit() = default;
  KeplerOrbit(const double p,
              const double e,
              const double i,
              const double lan,
              const double argpe,
              const double epoch,
              const double ta)
      : p(p), e(e), i(i), lan(lan), argpe(argpe), epoch(epoch), ta(ta) {}
  explicit KeplerOrbit(const StateVector& sv, double tol = 1e-8);

  double periapsisRadius() const { return p / (1.0 + e); }
  double apoapsisRadius() const { return p / (1.0 - e); }

  double apsisRadius(Apsis apsis) const;

  double semimajorAxis() const { return p / (1.0 - e * e); }

  double meanMotion(double mu) const;

  double period(const double mu) const {
    return 2.0 * std::numbers::pi / meanMotion(mu);
  }

  /// Calculate the position and velocity in the perifocal
  /// coordinate system PQW at an orbit's current true anomaly.
  std::pair<Vector3d, Vector3d> rvPqw(const KeplerBody& body) const;

  static Matrix3d pqwIjkMatrix(double i, double lan, double argpe);

  Matrix3d pqwIjkMatrix() const { return pqwIjkMatrix(i, lan, argpe); }

  /// Calculate the position and velocity in the body-centered
  /// equatorial coordinate system IJK at an orbit's current true
  /// anomaly.
  StateVector stateBci(const std::shared_ptr<KeplerBody>& body) const;
};

class KeplerVessel final : public RigidBody {
 public:
  StateVector propagate(double dt) const override;

 protected:
  std::pair<Vector3d, Vector3d> getIntermediateMoments(
      const StateVector& atSv,
      double tfrac,
      double dt) const override {
    return {};
  }
};

class KeplerBody final : public CelestialBody {
 public:
  /// Ephemerides at starting epoch
  KeplerOrbit ephem;
  /// Radius of this body's sphere of influence (`km`)
  double soi = 0.0;

  StateVector propagate(double dt) const;
};

std::optional<StateVector> keplerPropagate(const StateVector& sv,
                                           double dt,
                                           double tol = 1e-7,
                                           unsigned int maxIter = 500);

class KeplerPsys final : public Psys<KeplerBody> {
 public:
  std::shared_ptr<KeplerBody> get(const std::string_view name) const override {
    // TODO: find more efficient way to do this
    return bodies.at(std::string(name));
  }

  auto begin() const { return bodies.cbegin(); }
  auto end() const { return bodies.cend(); }

 private:
  friend void from_json(const nlohmann::json& j, KeplerPsys& psys);
  friend void to_json(nlohmann::json& j, const KeplerPsys& psys);
  std::unordered_map<std::string, std::shared_ptr<KeplerBody>> bodies{};
};

}  // namespace gravitas::physics