#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class DepthIntegrator: public Integrator {
public:
	DepthIntegrator(const PropertyList& props) {
		/* No parameters this time */
	}

	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {

		Color3f Lo(0.);

		// Find the surface that is visible in the requested direction 
		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return scene->getBackground(ray);

        Lo = Color3f(1.0 / its.t);

		return Lo;
	}

	std::string toString() const {
		return "Depth Integrator []";
	}
};

NORI_REGISTER_CLASS(DepthIntegrator, "depth");
NORI_NAMESPACE_END
