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

		EmitterQueryRecord emitterRecord(its.p);

		float pdflight;
		// Sample random light source
		const Emitter *random_emitter = scene->sampleEmitter(sampler->next1D(), pdflight);

		Color3f Le = random_emitter->sample(emitterRecord, sampler->next2D(), 0.);

		// Check if intersected material is emitter
		if(its.mesh->isEmitter()){
			emitterRecord.ref = ray.o;
			emitterRecord.wi = -ray.d;
			emitterRecord.n = its.shFrame.n;
			return its.mesh->getEmitter()->sample(emitterRecord, sampler->next2D(), 0.);
		}

		//Russian Roulette
		if (sampler->next1D() > 0.8){
			return Lo;
		}
		
		// Here perform a visibility query, to check whether the light 
		// source "em" is visible from the intersection point. 
		// For that, we create a ray object (shadow ray),
		// and compute the intersection
		
		Ray3f shadow_ray(its.p, emitterRecord.wi);
		// Ensures that the shadow ray does not overshoot the light source.
		shadow_ray.maxt = (emitterRecord.p - its.p).norm();
		Intersection shadowIntersection;
		if (!scene->rayIntersect(shadow_ray, shadowIntersection)){
			float pdfpositionlight = random_emitter->pdf(emitterRecord);
			BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d),its.toLocal(emitterRecord.wi), its.uv, ESolidAngle);
			Color3f fr = its.mesh->getBSDF()->eval(bsdfRecord);
			float cos_theta_i = its.shFrame.n.dot(emitterRecord.wi);	
			Le_acc += Le * fr * cos_theta_i / (pdflight*pdfpositionlight);
		}

		//BSDF
		BSDFQueryRecord bsdfRecord_samp(its.toLocal(-ray.d), its.uv);
		Color3f throughput = its.mesh->getBSDF()->sample(bsdfRecord_samp, sampler->next2D());
		Lo += throughput;


		Vector3f new_dir = its.toWorld(bsdfRecord_samp.wo);

		//reflect
        Ray3f rayR = Ray3f(its.p, new_dir);

		Intersection new_its;
		bool intersection = scene->rayIntersect(rayR, new_its);
		if(intersection && (new_its.mesh->isEmitter() && bsdfRecord_samp.measure != EDiscrete)) {		
			return Lo * Li(scene, sampler, rayR) / 0.8;
		}

		Lo += Le_acc;

		return Lo * Li(scene, sampler, rayR) / 0.8;

	}

	std::string toString() const {
		return "PathTracingNee[]";
	}
};

NORI_REGISTER_CLASS(PathTracingNEE, "path_nee");
NORI_NAMESPACE_END
