#include <gravitas/numerics.h>
#include <cmath>

using namespace gravitas::numerics;

pagmo::population GoldenSection::evolve(const pagmo::population& pop) const {
  if (pop.size() != 1) {
    return pop;
  }

  const auto& prob = pop.get_problem();
  const auto& [lbv, ubv] = prob.get_bounds();
  const auto plb = lbv[0];
  const auto pub = ubv[0];

  if (x0 < plb || x1 < plb || x2 < plb || x3 < plb || x0 > pub || x1 > pub ||
      x2 > pub || x3 > pub || !init) {
    x0 = plb;
    x1 = pub;

    const auto [curX, _] = select_individual(pop);
    const auto initEstimate = curX[0];
    const auto ieMin = initEstimate - plb;
    const auto maxIe = pub - initEstimate;
    if (std::abs(maxIe) > std::abs(ieMin)) {
      x1 = initEstimate;
      x2 = initEstimate + g2 * maxIe;
    } else {
      x1 = initEstimate - g2 * ieMin;
      x2 = initEstimate;
    }

    f1 = prob.fitness({x1})[0];
    f2 = prob.fitness({x2})[0];

    init = true;
  }

  if (f2 < f1) {
    x0 = x1;
    x1 = x2;
    x2 = g1 * x1 + g2 * x3;
    f1 = f2;
    f2 = prob.fitness({x2})[0];
  } else {
    x3 = x2;
    x2 = x1;
    x1 = g1 * x2 + g2 * x0;
    f2 = f1;
    f1 = prob.fitness({x1})[0];
  }

  auto newPop = pop;
  if (f1 < f2) {
    replace_individual(newPop, std::vector(1, x1), std::vector(1, f1));
  } else {
    replace_individual(newPop, std::vector(1, x2), std::vector(1, f2));
  }

  return newPop;
}
