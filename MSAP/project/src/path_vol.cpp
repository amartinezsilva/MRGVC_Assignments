#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/medium.h>
#include <nori/phaseFunction.h>
#include <iostream>
#include <cmath>

NORI_NAMESPACE_BEGIN

class PathTracingVol: public Integrator {
private:

public:
	PathTracingVol(const PropertyList& props) {
		/* No parameters this time */
	}

	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
		Color3f Lo(0.);
		Color3f Li_em(0.);
		Color3f medium_throughput(1.0f, 1.0f, 1.0f);
		//Ray3f pathRay(ray.o, ray.d);
		MediumSamplingRecord mRec;

		// Find the surface that is visible in the requested direction 
		Intersection its;

		bool validIntersection = scene->rayIntersect(ray, its);

		if (!validIntersection)
			return scene->getBackground(ray);


		float em_pdf = 0.0f, mat_pdf = 0.0f;
		float w_em = 0.0f, w_mat = 0.0f;


		if(validIntersection && its.mesh->hasMedium() &&
			
			its.mesh->getMedium()->sampleDistance(ray, mRec, sampler->next2D())){

			const PhaseFunction *phase = mRec.medium->getPhaseFunction();

			medium_throughput *= mRec.sigmaS * mRec.transmittance / mRec.pdfSuccess;	

			//Russian Roulette for extinction
			if (sampler->next1D() > 0.8){
				return Lo;
			}

			// //Inscattering

			//Sample emitter

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
				return medium_throughput * its.mesh->getEmitter()->sample(emitterRecord, sampler->next2D(), 0.);
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
				em_pdf = pdflight * pdfpositionlight;
				PhaseFunctionQueryRecord pRec(-ray.d,emitterRecord.wi, ESolidAngle); //to_local??
				float phaseVal = phase->eval(pRec);
				float cos_theta_i = its.shFrame.n.dot(emitterRecord.wi);	
				mat_pdf = pRec.pdf;

				
				if(mat_pdf + em_pdf > 0.0f) w_em = em_pdf / (mat_pdf + em_pdf);
				else w_em = em_pdf;
				
				Li_em +=  w_em * Le * cos_theta_i * Color3f(pRec.pdf)/ (pdflight*pdfpositionlight);
			}

			em_pdf = 0.0f, mat_pdf = 0.0f;

			//Sample phase function
			PhaseFunctionQueryRecord pRec_samp(-ray.d, Vector3f(0.0f), ESolidAngle); //to_local??
			float phaseVal = phase->sample(pRec_samp, sampler->next2D());
			Color3f throughput = Color3f(pRec_samp.pdf);
			mat_pdf = pRec_samp.pdf;
            medium_throughput *= phaseVal;

			Vector3f wo = Frame(-pRec_samp.wi).toWorld(pRec_samp.wo);
            Ray3f pathRay = Ray3f(mRec.p, wo);
            pathRay.mint = 0.0f;

			Intersection new_its;
			bool intersection = scene->rayIntersect(pathRay, new_its);
			Color3f Le_r(0.0f);

			// Conditions
			if (intersection && new_its.mesh->isEmitter()) {
				EmitterQueryRecord new_emitterRecord;
				new_emitterRecord.wi = pathRay.d;
				new_emitterRecord.n = new_its.shFrame.n;
				new_emitterRecord.dist = new_its.t;
				em_pdf = new_its.mesh->getEmitter()->pdf(new_emitterRecord);
			}

			if(mat_pdf + em_pdf > 0.0f) w_mat = mat_pdf / (mat_pdf + em_pdf);
			else w_mat = mat_pdf;

			// EMITTER SAMPLING contribution
			Lo += medium_throughput* Li_em / 0.8;

			// Phase function contribution
			Lo += medium_throughput * w_mat * throughput * Li(scene, sampler, pathRay) / 0.8;

			// // return Li(scene, sampler, pathRay) / 0.8;


		} else {
			//std::cout << "hola3"<< std::endl;
			if(its.mesh->hasMedium())
                medium_throughput *= mRec.transmittance / mRec.pdfFailure;


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
				return medium_throughput * its.mesh->getEmitter()->sample(emitterRecord, sampler->next2D(), 0.);
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
				em_pdf = pdflight * pdfpositionlight;
				BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d),its.toLocal(emitterRecord.wi), its.uv, ESolidAngle);
				Color3f fr = its.mesh->getBSDF()->eval(bsdfRecord);
				float cos_theta_i = its.shFrame.n.dot(emitterRecord.wi);	
				mat_pdf = its.mesh->getBSDF()->pdf(bsdfRecord);

				
				if(mat_pdf + em_pdf > 0.0f) w_em = em_pdf / (mat_pdf + em_pdf);
				else w_em = em_pdf;
				
				Li_em +=  w_em * Le * fr * cos_theta_i / (pdflight*pdfpositionlight);
			}

			em_pdf = 0.0f, mat_pdf = 0.0f;

			//BSDF
			BSDFQueryRecord bsdfRecord_samp(its.toLocal(-ray.d), its.uv);
			Color3f throughput = its.mesh->getBSDF()->sample(bsdfRecord_samp, sampler->next2D());
			mat_pdf = its.mesh->getBSDF()->pdf(bsdfRecord_samp);


			Vector3f new_dir = its.toWorld(bsdfRecord_samp.wo);

			//reflect
			Ray3f rayR = Ray3f(its.p, new_dir);

			Intersection new_its;
			bool intersection = scene->rayIntersect(rayR, new_its);
			Color3f Le_r(0.0f);


			// Conditions
			if (intersection && new_its.mesh->isEmitter()) {
				EmitterQueryRecord new_emitterRecord;
				new_emitterRecord.wi = rayR.d;
				new_emitterRecord.n = new_its.shFrame.n;
				new_emitterRecord.dist = new_its.t;
				em_pdf = new_its.mesh->getEmitter()->pdf(new_emitterRecord);
			}

			if(mat_pdf + em_pdf > 0.0f) w_mat = mat_pdf / (mat_pdf + em_pdf);
			else w_mat = mat_pdf;

			if (bsdfRecord_samp.measure == EDiscrete) {
				// BSDF contribution
				//return Li_em / 0.8;
				w_mat = 1.0 / (1.0 + em_pdf);
				return w_mat * throughput * Li(scene, sampler, rayR) / 0.8;
			}


			// EMITTER SAMPLING contribution
			Lo += medium_throughput* Li_em / 0.8;

			// BSDF contribution
			Lo += medium_throughput * w_mat * throughput * Li(scene, sampler, rayR) / 0.8;

		}


		return Lo;
	}

	std::string toString() const {
		return "PathTracingVol[]";
	}
};

NORI_REGISTER_CLASS(PathTracingVol, "path_vol");
NORI_NAMESPACE_END
