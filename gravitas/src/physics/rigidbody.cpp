#include <gravitas/physics/rigidbody.h>

using namespace gravitas::physics;

StateVector RigidBody::propagate(const double dt) const {
  return integrate_rk4(dt, 1, 0);
}

std::pair<Vector3d, Vector3d> RigidBody::getIntermediateMoments(
    const StateVector& atSv,
    double tfrac,
    double dt) const {
  Vector3d linAccel{}, angAccel{};

  linAccel += this->forces / this->mass;
  angAccel += this->angMom / this->mass;

  // TODO: Gravity
  return std::make_pair(linAccel, angAccel);
}

void RigidBody::addForce(const Vector3d& force, const Vector3d& at) {
  this->forces += force;
  this->angMom += force.cross(at);
}

Vector3d RigidBody::eulerKinematicInv(const Vector3d& torque,
                                      const Vector3d& angvel) const {
  return momentInv * (torque - angvel.cross(moment * angvel));
}

static Quaterniond angvelQuaternion(const Vector3d& angvel) {
  return Quaterniond(Eigen::AngleAxisd(angvel.norm(), angvel.normalized()));
}

StateVector RigidBody::integrate_rk2(const double h, const int nsub, const int isub) const {
  const auto h05 = h / 0.5;

  auto [accel0, torque0] = getIntermediateMoments(sv, 0.0, h);

  auto sv1 = sv;
  sv1.position += sv.velocity * h05;
  sv1.velocity += accel0 * h05;
  sv1.attitude = angvelQuaternion(h05 * sv.angvel) * sv.attitude;
  sv1.angvel += eulerKinematicInv(torque0, sv.angvel) * h05;

  auto [accel1, torque1] = getIntermediateMoments(sv1, (isub + 0.5) / nsub, h);

  auto svf = sv;
  svf.position += sv1.velocity * h;
  svf.velocity += accel1 * h;
  svf.attitude = angvelQuaternion(h * sv1.angvel) * sv.attitude;
  svf.angvel += eulerKinematicInv(torque1, sv1.angvel) * h;

  return svf;
}

StateVector RigidBody::integrate_rk4(double h, int nsub, int isub) const {
  const auto h05 = h / 0.5;
  const auto hi6 = h / 6.0;

  auto [accel0, torque0] = getIntermediateMoments(sv, 0.0, h);
  auto arot0 = eulerKinematicInv(torque0, sv.angvel);
  auto sv1 = sv;
  sv1.position += sv.velocity * h05;
  sv1.velocity += accel0 * h05;
  sv1.attitude = angvelQuaternion(h05 * sv.angvel) * sv.attitude;
  sv1.angvel += arot0 * h05;

  auto [accel1, torque1] = getIntermediateMoments(sv1, (isub + 0.5) / nsub, h);
  auto arot1 = eulerKinematicInv(torque1, sv1.angvel);
  auto sv2 = sv;
  sv2.position += sv.velocity * h05;
  sv2.velocity += accel1 * h05;
  sv2.attitude = angvelQuaternion(h05 * sv1.angvel) * sv.attitude;
  sv2.angvel += arot1 * h05;

  auto [accel2, torque2] = getIntermediateMoments(sv2, (isub + 0.5) / nsub, h);
  auto arot2 = eulerKinematicInv(torque2, sv2.angvel);
  auto sv3 = sv;
  sv3.position += sv.velocity * h;
  sv3.velocity += accel2 * h;
  sv3.attitude = angvelQuaternion(h * sv2.angvel) * sv.attitude;
  sv3.angvel += arot2 * h;

  auto [accel3, torque3] = getIntermediateMoments(sv3, (isub + 1.0) / nsub, h);
  auto arot3 = eulerKinematicInv(torque3, sv3.angvel);

  auto svf = sv;
  svf.position += (accel0 + (accel1 + accel2) * 2.0 + accel3) * hi6;
  svf.velocity +=
      (sv.velocity + (sv1.velocity + sv2.velocity) * 2.0 + sv3.velocity) * hi6;
  svf.attitude =
      angvelQuaternion(
          (sv.angvel + (sv1.angvel + sv2.angvel) * 2.0 + sv3.angvel) * hi6) *
      sv.attitude;
  svf.angvel += (arot0 + (arot1 + arot2) * 2.0 + arot3) * hi6;

  return svf;
}
