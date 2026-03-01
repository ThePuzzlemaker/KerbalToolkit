#include <gravitas/data.h>

using namespace gravitas;

void physics::from_json(const nlohmann::json& j, KeplerPsys& psys) {
  for (const auto& [name, jBody] : j.items()) {
    const auto kBody = std::make_shared<KeplerBody>();
    jBody.at("ephem").get_to(kBody->ephem);
    if (jBody.at("soi").is_null()) {
      kBody->soi = INFINITY;
    } else {
      jBody.at("soi").get_to(kBody->soi);
    }
    jBody.at("mu").get_to(kBody->mu);
    jBody.at("radius").get_to(kBody->radius);
    jBody.at("rotperiod").get_to(kBody->rotperiod);
    jBody.at("rotini").get_to(kBody->rotini);
    double angvel[3] = {0, 0, 0};
    jBody.at("angvel").get_to(angvel);
    kBody->angvel = Vector3d(angvel);
    jBody.at("isStar").get_to(kBody->isStar);
    kBody->name = name;
    psys.bodies[name] = kBody;
  }
  for (const auto& [name, jBody] : j.items()) {
    const auto& kBody = psys.bodies[name];
    if (!jBody.contains("parent") || jBody.at("parent").is_null()) {
      kBody->parent = {};
    } else {
      kBody->parent = psys.bodies[jBody.at("parent").get<std::string>()];
    }
    kBody->satellites = std::ranges::to<std::vector>(
        jBody.at("satellites") |
        std::views::transform([&](const auto& childName) {
          return std::static_pointer_cast<CelestialBody>(
              psys.bodies[childName]);
        }));
  }
}

void physics::to_json(nlohmann::json& j, const KeplerPsys& psys) {
  for (const auto& [name, body] : psys.bodies) {
    const nlohmann::json bodyJ{
        {"ephem", body->ephem},
        {"soi", body->soi},
        {"mu", body->mu},
        {"radius", body->radius},
        {"rotperiod", body->rotperiod},
        {"rotini", body->rotini},
        {"angvel", body->angvel},
        {"satellites",
         std::ranges::to<std::vector>(
             body->satellites |
             std::views::transform([](const auto& v) { return v->name; }))},
        {"parent",
         body->parent.transform([](const std::weak_ptr<CelestialBody>& v) {
           return v.lock()->name;
         })},
        {"isStar", body->isStar}};
    j[name] = bodyJ;
  }
}
