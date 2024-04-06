#include <brdf.h>
#include <utils.h>
#include <interaction.h>

/**
 * IdealDiffusion class
 */

IdealDiffusion::IdealDiffusion(vec3 diff) : reflectivity(diff) {}

vec3 IdealDiffusion::eval(const Interaction &interact) {
  // !!!DELETE THIS WHEN FINISHED
  UNIMPLEMENTED;
  return vec3::Zero();
}

Float IdealDiffusion::pdf(const Interaction &interact) {
  // !!!DELETE THIS WHEN FINISHED
  UNIMPLEMENTED;
  return 0;
}

Float IdealDiffusion::sample(Interaction &interact) {
  // !!!DELETE THIS WHEN FINISHED
  UNIMPLEMENTED;
  return 0;
}

bool IdealDiffusion::isDelta() const { return false; }

std::shared_ptr<BRDF> makeIdealDiffusion(const vec3 &diff) {
  return std::make_shared<IdealDiffusion>(diff);
}