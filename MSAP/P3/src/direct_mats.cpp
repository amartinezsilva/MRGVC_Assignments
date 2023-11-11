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

		// Find the surface that is visible in the requested direction 
		Color3f Lo(0.0f);
		Color3f Le(0.0f);
		Color3f Le_r(0.0f);

		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return scene->getBackground(ray);

		else if(its.mesh->isEmitter()){
			EmitterQueryRecord emitterRecord(its.mesh->getEmitter(), ray.o, its.p, its.shFrame.n, its.uv);
			Le = its.mesh->getEmitter()->eval(emitterRecord);
			return Le;
		}

		else{

			BSDFQueryRecord bRec(its.shFrame.toLocal(-ray.d), its.uv);
			const BSDF *its_bsdf = its.mesh->getBSDF();
			Color3f fr = its_bsdf->sample(bRec, sampler->next2D());
			float pdf_bsdf = its_bsdf->pdf(bRec);
			float cos_theta_i = its.shFrame.n.dot(bRec.wi);	

			//Cast a new ray and use emitter interface
			Ray3f bsdf_ray(its.p, its.shFrame.toWorld(bRec.wo));	

			// Find the surface that is visible in the requested direction 
			Intersection new_its;
			if (!scene->rayIntersect(bsdf_ray, new_its))
				return scene->getBackground(bsdf_ray);

			else if(new_its.mesh->isEmitter()){
				EmitterQueryRecord new_emitterRecord(new_its.mesh->getEmitter(), bsdf_ray.o, new_its.p, new_its.shFrame.n, new_its.uv);
				Le_r = new_its.mesh->getEmitter()->eval(new_emitterRecord);
			}
			
			return Le_r * its_bsdf->eval(bRec);
		}

	}

	std::string toString() const {
		return "DirectMaterialSampling[]";
	}
};

NORI_REGISTER_CLASS(DirectMaterialSampling, "direct_mats");
NORI_NAMESPACE_END
