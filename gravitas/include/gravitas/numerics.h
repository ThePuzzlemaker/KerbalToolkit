#pragma once

#include <pagmo/algorithms/not_population_based.hpp>

namespace gravitas::numerics {
/// Based on GoldenSectionSearch from argmin:
/// https://github.com/argmin-rs/argmin/blob/b3002851e70ee25ec03b626b2661f1c5bf033fae/crates/argmin/src/solver/goldensectionsearch/mod.rs
class GoldenSection final : public pagmo::not_population_based {
 public:
  GoldenSection()
      : x0(0.0), x1(0.0), x2(0.0), x3(0.0), f1(0.0), f2(0.0), init(false) {}

  [[nodiscard]] pagmo::population evolve(const pagmo::population& pop) const;

 private:
  constexpr static auto goldenRatio = 1.618'033'988'749'895;
  constexpr static auto g1 = -1.0 + goldenRatio;
  constexpr static auto g2 = 1.0 - g1;

  mutable double x0;
  mutable double x1;
  mutable double x2;
  mutable double x3;
  mutable double f1;
  mutable double f2;
  mutable bool init;
};

constexpr double modEuclid(const double a, const double b) {
  double r = std::fmod(a, b);
  if (r < 0)
    r += std::fabs(b);
  return r;
}
}  // namespace gravitas::numerics
