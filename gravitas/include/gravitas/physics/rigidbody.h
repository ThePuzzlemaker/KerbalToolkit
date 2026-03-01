#pragma once

#include <eigen3/Eigen/Geometry>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace gravitas::physics {
using std::operator""sv;

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

class CelestialBody;

class StateVector {
 public:
  /// Attitude
  Quaterniond attitude;
  /// Position (km)
  Vector3d position;
  /// Velocity (km/s)
  Vector3d velocity;
  /// Angular velocity, in angle-axis representation, i.e.,
  /// norm(angvel) has units rad/s
  Vector3d angvel;
  /// Parent body
  std::optional<std::weak_ptr<CelestialBody>> parent;
  /// Time tag (s)
  double time = 0.0f;

  std::shared_ptr<CelestialBody> expectParent() const {
    const auto p = parent.value().lock();
    if (!p)
      throw std::invalid_argument("StateVector::expectParent(): no parent");

    return p;
  }
};

class RigidBody {
 public:
  explicit RigidBody(const double mass, const Matrix3d& moi)
      : mass(mass), moment(moi), momentInv(moi.inverse()) {}

  RigidBody(const RigidBody&) = delete;
  RigidBody& operator=(const RigidBody&) = delete;
  RigidBody(RigidBody&&) = delete;
  RigidBody& operator=(RigidBody&&) = delete;

  virtual ~RigidBody() = default;

  const StateVector& stateVector() const { return sv; }

  void addForce(const Vector3d& force, const Vector3d& at);

  virtual StateVector propagate(double dt) const;

  /// The rigid body's mass (`kg`).
  double mass;

 protected:
  RigidBody() : mass(0.0) {}

  /// Returns linear and angular moments for the given state vector `sv`
  /// within a fraction `tfrac` of timestep `dt`.
  virtual std::pair<Vector3d, Vector3d> getIntermediateMoments(
      const StateVector& atSv,
      double tfrac,
      double dt) const;

  void update(const double dt) { sv = propagate(dt); }

  StateVector integrate_rk2(double h, int nsub, int isub) const;
  StateVector integrate_rk4(double h, int nsub, int isub) const;

 private:
  Matrix3d moment;
  Matrix3d momentInv;
  StateVector sv;

  Vector3d forces;
  Vector3d angMom;

  Vector3d eulerKinematicInv(const Vector3d& torque,
                             const Vector3d& angvel) const;
};

class CelestialBody {
 public:
  /// Standard gravitational parameter (`km^3/s^2`)
  double mu = 0.0;
  /// Mean radius of the body's sphere (`km`)
  double radius = 0.0;
  /// Rotational period, length of sidereal day (`sec`)
  double rotperiod = 0.0;
  /// Initial rotation about the body spin axis at given epoch
  /// (`rad`)
  double rotini = 0.0;
  /// Angular momentum direction in BCI coordinates.
  Vector3d angvel;
  /// Bodies orbiting this body.
  std::vector<std::shared_ptr<CelestialBody>> satellites;
  /// The parent body.
  std::optional<std::weak_ptr<CelestialBody>> parent;
  /// Name of this body.
  std::string name;
  /// Is this a star?
  bool isStar = false;

  std::shared_ptr<CelestialBody> expectParent() const {
    const auto p = parent.value().lock();
    if (!p)
      throw std::invalid_argument("Cel::expectParent(): no parent");

    return p;
  }
};

template <typename T>
  requires(std::is_base_of_v<CelestialBody, T>)
class Psys {
 public:
  virtual ~Psys() = default;

  virtual std::shared_ptr<T> get(std::string_view name) const = 0;
};

}  // namespace gravitas::physics
