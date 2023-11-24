#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <iostream>
#include <cmath>

NORI_NAMESPACE_BEGIN

class PathTracingNEE: public Integrator {
private:

public:
	PathTracingNEE(const PropertyList& props) {
		/* No parameters this time */
	}

	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {

		Color3f Lo(0.);
		Color3f Le_acc(0.);

		// Find the surface that is visible in the requested direction 
		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return scene->getBackground(ray);


		// Sample random light source
		EmitterQueryRecord emitterRecord(its.p);
		float pdflight;
		const Emitter *random_emitter = scene->sampleEmitter(sampler->next1D(), pdflight);
		Color3f Le = random_emitter->sample(emitterRecord, sampler->next2D(), 0.);

		// Check if intersected material is 
		// EMITTER
		if(its.mesh->isEmitter()){
			emitterRecord.ref = ray.o;
			emitterRecord.wi = -ray.d;
			emitterRecord.n = its.shFrame.n;
			return its.mesh->getEmitter()->sample(emitterRecord, sampler->next2D(), 0.);
		}

		float pdfpositionlight = random_emitter->pdf(emitterRecord);


		//  RUSSIAN ROULLETE 
		if (sampler->next1D() > 0.8) {
			return Lo;
		}


		//  SHADOW RAY
		Ray3f shadow_ray(its.p, emitterRecord.wi);
		// shadowRay.maxt is set to the distance between the intersection point its.p and the position of the light source emitterRecord.p, normalized by .norm().
		// This step ensures that the shadow ray's maximum length is set to the distance to the light source, so it won't overshoot the light source.
		shadow_ray.maxt = (emitterRecord.p - its.p).norm();
		Intersection shadowIntersection;
		if (!scene->rayIntersect(shadow_ray, shadowIntersection)){
			BSDFQueryRecord bsdfRecord_ems(its.toLocal(-ray.d),its.toLocal(emitterRecord.wi), its.uv, ESolidAngle);
        	Color3f fr = its.mesh->getBSDF()->eval(bsdfRecord_ems);
        	float cos_theta_i = its.shFrame.n.dot(emitterRecord.wi);
			Le_acc += Le * fr * cos_theta_i / (pdflight*pdfpositionlight);
		}

		///////////////////
		// CONTRIBUTIONS //
		///////////////////

		// Here perform a visibility query, to check whether the light 
		// source "em" is visible from the intersection point. 
		// For that, we create a ray object (shadow ray),
		// and compute the intersection	

		// BSDF calculation
		BSDFQueryRecord bsdfRecord_bsdf(its.toLocal(-ray.d), its.uv);
		Color3f throughput = its.mesh->getBSDF()->sample(bsdfRecord_bsdf, sampler->next2D());

		// EMITTER SAMPLING calculation
		Vector3f new_dir = its.toWorld(bsdfRecord_bsdf.wo);
        Ray3f rayR = Ray3f(its.p, new_dir);

		// For each light, we accomulate the incident light times the 
		// foreshortening times the BSDF term (i.e. the render equation). 


		Intersection new_its;
		bool intersection = scene->rayIntersect(rayR, new_its);


		// Conditions
		if (intersection && new_its.mesh->isEmitter()) {
			// EMITTER SAMPLING contribution
			return Le_acc / 0.8;
		}

		if (bsdfRecord_bsdf.measure == EDiscrete) {
			// BSDF contribution
			return throughput * Li(scene, sampler, rayR) / 0.8;
		}

		// EMITTER SAMPLING contribution
		Lo += Le_acc / 0.8;

		// BSDF contribution
		Lo += throughput * Li(scene, sampler, rayR) / 0.8;
	

		return Lo;
	}

	std::string toString() const {
		return "PathTracingNEE[]";
	}
};

NORI_REGISTER_CLASS(PathTracingNEE, "path_nee");
NORI_NAMESPACE_END