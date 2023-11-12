#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <iostream>
#include <cmath>

NORI_NAMESPACE_BEGIN

class DirectMIS: public Integrator {
private:

public:
	DirectMIS(const PropertyList& props) {
		/* No parameters this time */
	}

	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
		Color3f Lo(0.0f);
		
		
		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return scene->getBackground(ray);

		Color3f Le(0.0f);

		//Check if intersected material is Emitter
		if(its.mesh->isEmitter()){
			EmitterQueryRecord emitterRecord;
			emitterRecord.wi = ray.d;
			emitterRecord.n = its.shFrame.n;
			Le = its.mesh->getEmitter()->eval(emitterRecord);
			//Lo += Le;
			return Le;
		}
		
		// -------------------------------- MIS PART (FILL HERE) -------------------------------------------------------//

		float em_pdf, mat_pdf;
		float w_em, w_mat;

		// ----------- F_ems part -------------------//

		//Sample emitter
		EmitterQueryRecord emitterRecord_ems(its.p);
		float pdflight, pdfpositionlight;
		const Emitter *random_emitter = scene->sampleEmitter(sampler->next1D(), pdflight);
		Color3f Li_em = random_emitter->sample(emitterRecord_ems, sampler->next2D(), 0.);
		pdfpositionlight = random_emitter->pdf(emitterRecord_ems);
		
		em_pdf = pdflight * pdfpositionlight; // ??? //

		float cos_theta_i = Frame::cosTheta(its.shFrame.toLocal(emitterRecord_ems.wi));

		// Visibility query
		Ray3f shadow_ray(its.p, emitterRecord_ems.wi);
		// Ensures that the shadow ray does not overshoot the light source.
		shadow_ray.maxt = (emitterRecord_ems.p - its.p).norm();
		Intersection shadowIntersection;
		if (scene->rayIntersect(shadow_ray, shadowIntersection)){
			Li_em = Color3f(0.0f);
		}

		//Evaluate pdf of material sampling using a sample generated with emitter sampling
		BSDFQueryRecord bsdfRecord_ems(its.toLocal(-ray.d), its.toLocal(emitterRecord_ems.wi), its.uv, ESolidAngle);
		mat_pdf = its.mesh->getBSDF()->pdf(bsdfRecord_ems);
		Color3f f = its.mesh->getBSDF()->eval(bsdfRecord_ems);

		// Weights
		w_em = em_pdf / (em_pdf + mat_pdf);

		//Emitter mat
		Color3f F_ems = Li_em * f * cos_theta_i / em_pdf;
		Lo += w_em * F_ems;


		// ----------- F_mat part -------------------//

		//Sample BSDF
		BSDFQueryRecord bsdfRecord_mat(its.toLocal(-ray.d), its.uv);
		Color3f throughput = its.mesh->getBSDF()->sample(bsdfRecord_mat, sampler->next2D());

		mat_pdf = its.mesh->getBSDF()->pdf(bsdfRecord_mat);


		//reflect
		Vector3f new_dir = its.toWorld(bsdfRecord_mat.wo);
        Ray3f rayR = Ray3f(its.p, new_dir);

		Intersection new_its;
		Color3f Le_r(0.0f);

		bool intersection = scene->rayIntersect(rayR, new_its);
		if (!intersection) {
			Le_r = scene->getBackground(rayR);
		} else if(new_its.mesh->isEmitter()) {
			//std::cout <<"emitterrr" << endl;
			EmitterQueryRecord new_emitterRecord;
			new_emitterRecord.wi = rayR.d;
			new_emitterRecord.n = new_its.shFrame.n;
			Le_r = new_its.mesh->getEmitter()->eval(new_emitterRecord);
			//Evaluate pdf of emitter sampling using a sample generated with material sampling
			em_pdf = new_its.mesh->getEmitter()->pdf(new_emitterRecord);
		}

		// Weights
		w_mat = mat_pdf / (em_pdf + mat_pdf);

		//Emitter mat
		Color3f F_mat = Le_r * throughput;
		Lo += w_mat * F_mat;

		
		// // -------------------------------- EMS PART -------------------------------------------------------//

		// EmitterQueryRecord emitterRecord(its.p);

		// float pdflight;
		// // Sample random light source
		// const Emitter *random_emitter = scene->sampleEmitter(sampler->next1D(), pdflight);

		// Color3f Le = random_emitter->sample(emitterRecord, sampler->next2D(), 0.);

		// // Check if intersected material is emitter
		// if(its.mesh->isEmitter()){
		// 	emitterRecord.ref = ray.o;
		// 	emitterRecord.wi = -ray.d;
		// 	emitterRecord.n = its.shFrame.n;
		// 	return its.mesh->getEmitter()->sample(emitterRecord, sampler->next2D(), 0.);
		// }

		
		// // Here perform a visibility query, to check whether the light 
		// // source "em" is visible from the intersection point. 
		// // For that, we create a ray object (shadow ray),
		// // and compute the intersection
		
		// Ray3f shadow_ray(its.p, emitterRecord.wi);
		// // Ensures that the shadow ray does not overshoot the light source.
		// shadow_ray.maxt = (emitterRecord.p - its.p).norm();
		// Intersection shadowIntersection;
		// if (scene->rayIntersect(shadow_ray, shadowIntersection)){
		// 	return Lo;
		// }

		// float pdfpositionlight = random_emitter->pdf(emitterRecord);
 
		// BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d),its.toLocal(emitterRecord.wi), its.uv, ESolidAngle);
        // Color3f fr = its.mesh->getBSDF()->eval(bsdfRecord);
        // float cos_theta_i = its.shFrame.n.dot(emitterRecord.wi);	

		// // For each light, we accomulate the incident light times the 
		// // foreshortening times the BSDF term (i.e. the render equation). 
		// Lo += Le * fr * cos_theta_i / (pdflight*pdfpositionlight);

		// // ------------------------  MATS PART -----------------------------------------//
		
		// //EMITTER
		// if(its.mesh->isEmitter()){
		// 	EmitterQueryRecord emitterRecord;
		// 	emitterRecord.wi = ray.d;
		// 	emitterRecord.n = its.shFrame.n;
		// 	return its.mesh->getEmitter()->eval(emitterRecord);
		// }


		// //BSDF
		// BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d), its.uv);
		// Color3f throughput = its.mesh->getBSDF()->sample(bsdfRecord, sampler->next2D());

		// Vector3f new_dir = its.toWorld(bsdfRecord.wo);

		// //reflect
        // Ray3f rayR = Ray3f(its.p, new_dir);

		// Intersection new_its;
		// Color3f Le_r(0.0f);

		// bool intersection = scene->rayIntersect(rayR, new_its);
		// if (!intersection) {
		// 	Le_r = scene->getBackground(rayR);
		// } else if(new_its.mesh->isEmitter()) {
		// 	//std::cout <<"emitterrr" << endl;
		// 	EmitterQueryRecord new_emitterRecord;
		// 	new_emitterRecord.wi = rayR.d;
		// 	new_emitterRecord.n = new_its.shFrame.n;
		// 	Le_r = new_its.mesh->getEmitter()->eval(new_emitterRecord);	
		// }
			
		// Lo += Le_r * throughput;


		return Lo;
	}

	std::string toString() const {
		return "DirectMIS[]";
	}
};

NORI_REGISTER_CLASS(DirectMIS, "direct_mis");
NORI_NAMESPACE_END
