// #include <nori/warp.h>
// #include <nori/integrator.h>
// #include <nori/scene.h>
// #include <nori/emitter.h>
// #include <nori/bsdf.h>
// #include <nori/ray.h>

// NORI_NAMESPACE_BEGIN

// class PathTracingNEE : public Integrator {
// private:
// 	const double P_NO_ABSORTION = 0.95;

// 	// lightHit_back is used in the previous recursive call
// 	Color3f Li_rec(const Scene* scene, Sampler* sampler, const Ray3f& ray, bool& lightHit_back) const {
// 		Intersection its;
// 		if (!scene->rayIntersect(ray, its)) {
// 			lightHit_back = true;
// 			return scene->getBackground(ray);
// 		}
// 		if (its.mesh->isEmitter()) {	// Account for visible light sources
// 			EmitterQueryRecord emR;
// 			emR.wi = ray.d;
// 			emR.n = its.shFrame.n;
// 			lightHit_back = true;
// 			return its.mesh->getEmitter()->eval(emR);
// 		}	
// 		lightHit_back = false;

// 		// -------------- Generating next step with BSDF sampling --------------
// 		BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d));
// 		Color3f bsdf_sampled = its.mesh->getBSDF()->sample(bsdfRecord, sampler->next2D());
// 		Ray3f newRay(its.p, its.toWorld(bsdfRecord.wo));

// 		bool lightHit_next = false;		// The ray hits a light in the next step
// 		Color3f Li;
// 		if (sampler->next1D() < P_NO_ABSORTION) {
// 			Li = Li_rec(scene, sampler, newRay, lightHit_next) / P_NO_ABSORTION;
// 		} else {
// 			Li = Color3f(0.0f);
// 		} 

// 		// ---------------------- Next event estimation -----------------------
// 		Color3f Nee(0.0);
// 		if (bsdfRecord.measure != EDiscrete) {
// 			float pdflight;
// 			const Emitter* em = scene->sampleEmitter(sampler->next1D(), pdflight);

// 			EmitterQueryRecord emitterRecord(its.p);
// 			Color3f Le_emitter = em->sample(emitterRecord, sampler->next2D(), 0.);

// 			BSDFQueryRecord bsdfRecord_eval(its.toLocal(-ray.d),
// 								its.toLocal(emitterRecord.wi), its.uv, ESolidAngle);
// 			auto throughput = its.mesh->getBSDF()->eval(bsdfRecord_eval);

// 			// Visibility query
// 			Ray3f shadowRay(its.p, emitterRecord.wi);
// 			Intersection its_shadow;
// 			bool intersect_shadow = scene->rayIntersect(shadowRay, its_shadow);
// 			if (!intersect_shadow || its_shadow.t > (emitterRecord.dist - Epsilon)) {
// 				Nee = Le_emitter * its.shFrame.n.dot(emitterRecord.wi) * throughput
// 					/ pdflight;
// 			}
// 		}

// 		// -------------------- Mixing both contributions ---------------------
// 		if (bsdfRecord.measure == EDiscrete) {
// 			return Li * bsdf_sampled;
// 		}
// 		if (lightHit_next) {
// 			return Nee;
// 		} else {
// 			return Nee + Li * bsdf_sampled;
// 		}
// 	}

// public:
// 	PathTracingNEE(const PropertyList& props) {
// 		/* No parameters this time */
// 	}

// 	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
// 		bool unused;
// 		return Li_rec(scene, sampler, ray, unused);
// 	}

// 	std::string toString() const {
// 		return "Path tracing NEE []";
// 	}
// };

// NORI_REGISTER_CLASS(PathTracingNEE, "path_nee") ;
// NORI_NAMESPACE_END