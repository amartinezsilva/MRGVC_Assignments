#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <iostream>
#include <cmath>

NORI_NAMESPACE_BEGIN

class DirectMaterialSampling: public Integrator {
private:

public:
	DirectMaterialSampling(const PropertyList& props) {
		/* No parameters this time */
	}

	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {

		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return Color3f(0.0f);


		//EMITTER
		if(its.mesh->isEmitter()){
			EmitterQueryRecord emitterRecord(its.mesh->getEmitter(), ray.o, its.p, its.shFrame.n, its.uv);
			return its.mesh->getEmitter()->eval(emitterRecord);
		}


		//BSDF
		BSDFQueryRecord bsdfRecord(its.shFrame.toLocal(-ray.d));
		its.mesh->getBSDF()->sample(bsdfRecord, sampler->next2D());
		Vector3f new_dir = its.shFrame.toWorld(bsdfRecord.wo);

		//reflect
        Ray3f rayR = Ray3f(its.p, new_dir);

		Color3f throughput = its.mesh->getBSDF()->eval(bsdfRecord);
		Color3f Li = DirectMaterialSampling::Li(scene, sampler, rayR);
		
		return  throughput * Li;
	}

	std::string toString() const {
		return "DirectMaterialSampling[]";
	}
};

NORI_REGISTER_CLASS(DirectMaterialSampling, "direct_mats");
NORI_NAMESPACE_END
