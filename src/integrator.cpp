#include <integrator.h>
#include <brdf.h>
#include <light.h>

/**
 * Integrator class
 */
Integrator::Integrator(std::shared_ptr<Camera> camera) : camera(camera) {}

/**
 * PhongLightingIntegrator class
 */
PathIntegrator::PathIntegrator(std::shared_ptr<Camera> camera)
    : Integrator(camera) {}

/**
 * render a scene
 * @param[in] the given scene
 */
void PathIntegrator::render(Scene &scene) {
  int now = 0;
#ifdef USE_OPENMP
#pragma omp parallel for schedule(guided, 2) default(none) shared(now)
#endif
  for (int dx = 0; dx < camera->getFilm().resolution.x(); ++dx) {
#ifdef USE_OPENMP
#pragma omp atomic
#endif
    ++now;
    printf("\r%.02f%%", now * 100.0 / camera->getFilm().resolution.x());
    for (int dy = 0; dy < camera->getFilm().resolution.y(); ++dy) {
      vec3 L(0, 0, 0);

      // TODO: anti-aliasing

      Ray ray = camera->generateRay(dx, dy);
      L += radiance(scene, ray);
      camera->setPixel(dx, dy, L);
    }
  }
}

/**
 * calculate the radiance with given scene, ray and interaction
 * @param[in] scene the given scene
 * @param[in] interaction the given interaction
 * @param[in] the given ray
 */
vec3 PathIntegrator::radiance(Scene &scene, const Ray &ray) const {
  vec3 L(0, 0, 0);
  // DELETE these lines
  // ---------------
  Interaction interaction;
  if (!scene.intersect(ray, interaction)) return L;
  L = (interaction.normal + vec3(1, 1, 1)) / 2;
  if (interaction.type == Interaction::Type::LIGHT)
    L = scene.getLight()->getRadiance();
  // ---------------
  return L;
}

std::shared_ptr<Integrator> makePathIntegrator(std::shared_ptr<Camera> camera) {
  return std::make_shared<PathIntegrator>(camera);
}
