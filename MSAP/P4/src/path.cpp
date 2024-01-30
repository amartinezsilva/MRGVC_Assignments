#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <iostream>
#include <cmath>

NORI_NAMESPACE_BEGIN

class PathTracing: public Integrator {
private:

public:
	PathTracing(const PropertyList& props) {
		/* No parameters this time */
	}

	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
		Color3f Lo(0.0);

		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return scene->getBackground(ray);


		//EMITTER
		if(its.mesh->isEmitter()){
			EmitterQueryRecord emitterRecord(its.p);
			emitterRecord.wi = ray.d;
			emitterRecord.n = its.shFrame.n;
			return its.mesh->getEmitter()->eval(emitterRecord);
		}

		//Russian Roulette
		if (sampler->next1D() > 0.8){
			return Lo;
		}

		//BSDF
		BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d), its.uv);
		Color3f throughput = its.mesh->getBSDF()->sample(bsdfRecord, sampler->next2D());

		Vector3f new_dir = its.toWorld(bsdfRecord.wo);

		//reflect
        Ray3f rayR = Ray3f(its.p, new_dir);

		return throughput * Li(scene, sampler, rayR) / 0.8;
	}

	std::string toString() const {
		return "PathTracing[]";
	}
};

NORI_REGISTER_CLASS(PathTracing, "path");
NORI_NAMESPACE_END
