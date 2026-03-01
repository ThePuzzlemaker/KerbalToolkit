#pragma once

#include <nlohmann/json.hpp>
#include "physics/kepler.h"

namespace gravitas::physics {
// NOLINTNEXTLINE(*-redundant-declaration)
void from_json(const nlohmann::json& j, KeplerPsys& psys);
// NOLINTNEXTLINE(*-redundant-declaration)
void to_json(nlohmann::json& j, const KeplerPsys& psys);
}  // namespace gravitas::physics
