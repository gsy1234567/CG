#include <light.h>
#include <utils.h>

/**
 * Light class
 */
Light::Light(const vec3 &position, const vec3 &radiance)
    : position(position), radiance(radiance) {}

vec3 Light::getPosition() const { return position; }

vec3 Light::getRadiance() const { return radiance; }

/**
 * AreaLight class
 */
AreaLight::AreaLight(const vec3 &position, const vec3 &color, const vec2 &size)
    : Light(position, color),
      areaSize(size),
      geoms(makeParallelogram(position - vec3(size[0], 0, size[1]) / 2,
                              vec3(size[0], 0, 0), vec3(0, 0, size[1]),
                              vec3(0, -1, 0), nullptr)) {}

vec3 AreaLight::emission(vec3 pos, vec3 dir) {
  // !!!DELETE THIS WHEN FINISHED
  UNIMPLEMENTED;
  return vec3::Zero();
}

Float AreaLight::pdf(const Interaction &ref_it, vec3 pos) {
  // !!!DELETE THIS WHEN FINISHED
  UNIMPLEMENTED;
  return 0;
}

vec3 AreaLight::sample(Interaction &refIt, Float *pdf) {
  // !!!DELETE THIS WHEN FINISHED
  UNIMPLEMENTED;
  return vec3::Zero();
}

bool AreaLight::intersect(Interaction &interaction, const Ray &ray) const {
  bool intersection = false;
  for (auto &i : geoms)
    intersection = intersection || i->intersect(interaction, ray);
  interaction.type = Interaction::Type::LIGHT;
  return intersection;
}

std::shared_ptr<Light> makeAreaLight(const vec3 &position, const vec3 &color,
                                     const vec2 &size) {
  return std::make_shared<AreaLight>(position, color, size);
}