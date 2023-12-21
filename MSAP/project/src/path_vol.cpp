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
		Ray3f pathRay(ray.o, ray.d);
		MediumSamplingRecord mRec;

		// Find the surface that is visible in the requested direction 
		Intersection its;

		bool validIntersection = scene->rayIntersect(ray, its);

		if (!scene->rayIntersect(ray, its))
			return scene->getBackground(ray);


		float em_pdf = 0.0f, mat_pdf = 0.0f;
		float w_em = 0.0f, w_mat = 0.0f;


				// ALREADY INSIDE MEDIUM
		// std::vector<Medium *> medium = scene->getMedia();
		// medium[0]->eval(ray, mRec);
		// PhaseFunctionQueryRecord pRec(-ray.d, Vector3f(0.0f), ESolidAngle);
		// medium[0]->getPhaseFunction()->sample(pRec, sampler->next2D());
		// std::string summary = medium[0]->toString();
		// std::cout << summary << std::endl;
		// std::cout << "phase function: " << phaseVal;
		// MediumSamplingRecord mRec;
		// medium[0]->eval(ray, mRec);


				// creo que ahora lo que hay que hacer es quitar esto

		// la idea creo que es, mediante montecarlo cojo un punto de mi rayo. COMO? no se
		// a toda la L, pase lo que pase le tengo que añadir el transmitance desde la camara ray.mixt hasta ese punto
		// posteriormente con el inscattering
		// 		- tengo que añadir el transmitance desde el punto hasta el emitter, con el pdf del emiter
		// 		- y por otro ladotengo que crear una nueva direccion con wo
		//			- si es llego a un emmiter devuelvo Lmat, con el transmitance desde el punto hasta el emitter y la phase function the heyseygreeinstein
		// por otro lado con direct light
		//      - simplemente transmitance desde el punto de en el que ha chocado con el objeto hasta el no se exactamente donde
		//      - simplemente transmitance desde el punto de en el que ha chocado con el objeto hasta el emitter


		if(validIntersection && its.mesh->hasMedium() &&
			its.mesh->getMedium()->sampleDistance(Ray3f(pathRay, 0.0, its.t), mRec, sampler->next2D())){

			//const Medium *medium = its.mesh->getMedium();
			const PhaseFunction *phase = mRec.medium->getPhaseFunction();
			

			medium_throughput *= mRec.sigmaS * mRec.transmittance / mRec.pdfSuccess;	

			PhaseFunctionQueryRecord pRec(-ray.d, Vector3f(0.0f), ESolidAngle);

			float phaseVal = phase->sample(pRec, sampler->next2D());
            medium_throughput *= phaseVal;

			Vector3f wo = Frame(-pRec.wi).toWorld(pRec.wo);
            pathRay = Ray3f(mRec.p, wo);
            pathRay.mint = 0.0f;
            validIntersection = scene->rayIntersect(pathRay, its);


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
