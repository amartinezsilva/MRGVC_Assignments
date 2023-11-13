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
		Color3f Lo(0.0);

		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return scene->getBackground(ray);


		//EMITTER
		if(its.mesh->isEmitter()){
			EmitterQueryRecord emitterRecord;
			emitterRecord.wi = ray.d;
			emitterRecord.n = its.shFrame.n;
			return its.mesh->getEmitter()->eval(emitterRecord);
		}


		//BSDF
		BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d), its.uv);
		Color3f throughput = its.mesh->getBSDF()->sample(bsdfRecord, sampler->next2D());

		Vector3f new_dir = its.toWorld(bsdfRecord.wo);

		//reflect
        Ray3f rayR = Ray3f(its.p, new_dir);

		Intersection new_its;
		Color3f Le_r(0.0f);

		bool intersection = scene->rayIntersect(rayR, new_its);
		if (!intersection) {
			Le_r = scene->getBackground(rayR);
		} else if(new_its.mesh->isEmitter()) {
			EmitterQueryRecord new_emitterRecord;
			new_emitterRecord.wi = rayR.d;
			new_emitterRecord.n = new_its.shFrame.n;
			Le_r = new_its.mesh->getEmitter()->eval(new_emitterRecord);	
		}
			
		Lo += Le_r * throughput;
		return Lo;
	}

	std::string toString() const {
		return "DirectMaterialSampling[]";
	}
};

NORI_REGISTER_CLASS(DirectMaterialSampling, "direct_mats");
NORI_NAMESPACE_END
