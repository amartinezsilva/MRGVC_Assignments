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
			return Le;
		}
		
		// -------------------------------- MIS PART (FILL HERE) -------------------------------------------------------//

		float em_pdf = 0.0f, mat_pdf;
		float w_em = 0.0f, w_mat = 0.0f;

		// ----------- F_mat part -------------------//
		Color3f F_mat(0.0f);
		//Sample BSDF
		BSDFQueryRecord bsdfRecord_mat(its.toLocal(-ray.d), its.uv);
		Color3f throughput = its.mesh->getBSDF()->sample(bsdfRecord_mat, sampler->next2D());

		mat_pdf = its.mesh->getBSDF()->pdf(bsdfRecord_mat);

		//reflect
		Vector3f new_dir = its.toWorld(bsdfRecord_mat.wo);
        Ray3f rayR = Ray3f(its.p, new_dir);

		Intersection new_its;
		Color3f Le_r(0.0f);
		bool scene_intersects = scene->rayIntersect(rayR, new_its);
		if (!scene_intersects) {
			Le_r = scene->getBackground(rayR);
		} else if(new_its.mesh->isEmitter()) {
			//std::cout <<"emitterrr" << endl;
			EmitterQueryRecord new_emitterRecord;
			new_emitterRecord.wi = rayR.d;
			new_emitterRecord.n = new_its.shFrame.n;
			new_emitterRecord.dist = new_its.t;
			Le_r = new_its.mesh->getEmitter()->eval(new_emitterRecord);
			//Evaluate pdf of emitter sampling using a sample generated with material sampling
			em_pdf = new_its.mesh->getEmitter()->pdf(new_emitterRecord);
		}

		F_mat = Le_r * throughput;
		// Weights material sampling part
		if(mat_pdf + em_pdf > 0.0f) w_mat = mat_pdf / (mat_pdf + em_pdf);
		else w_mat = mat_pdf;

		Color3f L_mat = w_mat * F_mat;
		Lo += L_mat;

		// ----------- F_ems part -------------------//
		mat_pdf = 0.0f;
		Color3f F_ems(0.0f);
		//Sample emitter
		EmitterQueryRecord emitterRecord_ems(its.p);
		float pdflight, pdfpositionlight;
		const Emitter *random_emitter = scene->sampleEmitter(sampler->next1D(), pdflight);
		Color3f Li_em = random_emitter->sample(emitterRecord_ems, sampler->next2D(), 0.);
		pdfpositionlight = random_emitter->pdf(emitterRecord_ems);
		
		em_pdf = pdflight * pdfpositionlight;

		float cos_theta_i = its.shFrame.n.dot(emitterRecord_ems.wi);

		// Visibility query
		Ray3f shadow_ray(its.p, emitterRecord_ems.wi);
		// Ensures that the shadow ray does not overshoot the light source.
		shadow_ray.maxt = (emitterRecord_ems.p - its.p).norm();
		Intersection shadowIntersection;
		bool shadow_intersects = scene->rayIntersect(shadow_ray, shadowIntersection);
		if(!shadow_intersects){		
			//Evaluate pdf of material sampling using a sample generated with emitter sampling
			BSDFQueryRecord bsdfRecord_ems(its.toLocal(-ray.d), its.toLocal(emitterRecord_ems.wi), its.uv, ESolidAngle);
			mat_pdf = its.mesh->getBSDF()->pdf(bsdfRecord_ems);
			Color3f f = its.mesh->getBSDF()->eval(bsdfRecord_ems);
			//Emitter mat
			F_ems = Li_em * f * cos_theta_i / em_pdf;
		}

		// Weights material sampling part
		if(mat_pdf + em_pdf > 0.0f) w_em = em_pdf / (mat_pdf + em_pdf);
		else w_em = em_pdf;

		Color3f L_em = w_em * F_ems;

		Lo += L_em;

		return Lo;
	}

	std::string toString() const {
		return "DirectMIS[]";
	}
};

NORI_REGISTER_CLASS(DirectMIS, "direct_mis");
NORI_NAMESPACE_END
