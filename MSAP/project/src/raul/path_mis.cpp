// #include <nori/warp.h>
// #include <nori/integrator.h>
// #include <nori/scene.h>
// #include <nori/emitter.h>
// #include <nori/bsdf.h>
// #include <nori/ray.h>

// NORI_NAMESPACE_BEGIN

// class PathTracingMIS : public Integrator {
// private:
// 	const double P_NO_ABSORTION = 0.95;

// 	// pdf_em_back is used in the previous recursive call
// 	Color3f Li_rec(const Scene* scene, Sampler* sampler, const Ray3f& ray, float& pdf_em_back) const {
// 		Color3f radiance(0.0);

// 		Intersection its;
// 		if (!scene->rayIntersect(ray, its)) {
// 			pdf_em_back = scene->getPdfBackground(ray);
// 			return scene->getBackground(ray);
// 		}
// 		if (its.mesh->isEmitter()) {	// Account for visible light sources
// 			EmitterQueryRecord emR;
// 			emR.wi = ray.d;
// 			emR.n = its.shFrame.n;
// 			emR.dist = its.t;
// 			emR.p = its.p;

// 			pdf_em_back = its.mesh->getEmitter()->pdf(emR);
// 			return its.mesh->getEmitter()->eval(emR);
// 		}	
// 		pdf_em_back = 0;

// 		// NEE, but needed for BSDF sampling
// 		float pdflight;
// 		const Emitter* em = scene->sampleEmitter(sampler->next1D(), pdflight);

// 		// -------------- Generating next step with BSDF sampling --------------
// 		BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d));
// 		Color3f bsdf_sampled = its.mesh->getBSDF()->sample(bsdfRecord, sampler->next2D());
// 		Ray3f newRay(its.p, its.toWorld(bsdfRecord.wo));

// 		float pdf_em_bsdfDir = 0;
// 		if (sampler->next1D() < P_NO_ABSORTION) {
// 			Color3f Li = Li_rec(scene, sampler, newRay, pdf_em_bsdfDir) / P_NO_ABSORTION;

// 			float w_bsdf;
// 			if (bsdfRecord.measure == EDiscrete) {
// 				w_bsdf = 1.0;
// 			} else {
// 				w_bsdf = its.mesh->getBSDF()->pdf(bsdfRecord);
// 			}
// 			if (w_bsdf > Epsilon) {	 // Avoiding -nans
// 				w_bsdf /= w_bsdf + (pdflight * pdf_em_bsdfDir);
// 			}
// 			radiance += Li * bsdf_sampled * w_bsdf;
// 		}

// 		// ---------------------- Next event estimation -----------------------
// 		if (bsdfRecord.measure != EDiscrete) {
// 			EmitterQueryRecord emitterRecord(its.p);
// 			Color3f Le_emitter = em->sample(emitterRecord, sampler->next2D(), 0.);

// 			BSDFQueryRecord bsdfRecord_eval(its.toLocal(-ray.d),
// 								its.toLocal(emitterRecord.wi), its.uv, ESolidAngle);
// 			auto throughput = its.mesh->getBSDF()->eval(bsdfRecord_eval);
// 			float pdf_bsdf_emDir = its.mesh->getBSDF()->pdf(bsdfRecord_eval);

// 			// Visibility query
// 			Ray3f shadowRay(its.p, emitterRecord.wi);
// 			Intersection its_shadow;
// 			bool intersect_shadow = scene->rayIntersect(shadowRay, its_shadow);
// 			if (!intersect_shadow || its_shadow.t > (emitterRecord.dist - Epsilon)) {
// 				float w_em = pdflight * em->pdf(emitterRecord);
// 				if (w_em > Epsilon) {	 // Avoiding -nans
// 					w_em /= w_em + pdf_bsdf_emDir;
// 				}
				
// 				radiance += w_em * Le_emitter * its.shFrame.n.dot(emitterRecord.wi) * throughput
// 					  		/ pdflight;
// 			}
// 		}

// 		return radiance;	
// 	}

// public:
// 	PathTracingMIS(const PropertyList& props) {
// 		/* No parameters this time */
// 	}

// 	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
// 		float unused;
// 		return Li_rec(scene, sampler, ray, unused);
// 	}

// 	std::string toString() const {
// 		return "Path tracing MIS []";
// 	}
// };

// NORI_REGISTER_CLASS(PathTracingMIS, "path_mis") ;
// NORI_NAMESPACE_END