#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/ray.h>

NORI_NAMESPACE_BEGIN

class PathTracingVOL : public Integrator {
private:
	const double P_NO_ABSORTION = 0.95;

	// pdf_em_back is used in the previous recursive call
	Color3f Li_rec(const Scene* scene, Sampler* sampler, const Ray3f& ray, float& pdf_em_back) const {
		Color3f radiance(0.0);
		Color3f medium_throughput(1.0f, 1.0f, 1.0f);
		MediumSamplingRecord mRec;

		Intersection its;

		bool validIntersection = scene->rayIntersect(ray, its);

		if (!validIntersection) {
			pdf_em_back = scene->getPdfBackground(ray);
			return scene->getBackground(ray);
		}

		//std::vector<Medium *> medium = scene->getMedia();  //uncomment if the medium is on the whole scene


		if(validIntersection &&	
			its.mesh->hasMedium() && its.mesh->getMedium()->sampleDistance(ray, mRec, sampler->next2D())){   //uncomment if the medium is contained in a mesh
			//medium[0]->sampleDistance(ray, mRec, sampler->next2D())){ //uncomment if the medium is over all the scene
		
			const PhaseFunction *phase = mRec.medium->getPhaseFunction();
			medium_throughput *= mRec.sigmaS * mRec.transmittance / mRec.pdfSuccess;	

			if (its.mesh->isEmitter()) {	// Account for visible light sources
				EmitterQueryRecord emR;
				emR.wi = ray.d;
				emR.n = its.shFrame.n;
				emR.dist = its.t;
				emR.p = its.p;

				pdf_em_back = its.mesh->getEmitter()->pdf(emR);
				return medium_throughput * its.mesh->getEmitter()->eval(emR);
			}	
			pdf_em_back = 0;

			// NEE, but needed for BSDF sampling
			float pdflight;
			const Emitter* em = scene->sampleEmitter(sampler->next1D(), pdflight);

			// -------------- Generating next step with BSDF sampling --------------
			PhaseFunctionQueryRecord pRec_samp(its.toLocal(-ray.d), Vector3f(0.0f), ESolidAngle); //to_local??
			float phase_sampled = phase->sample(pRec_samp, sampler->next2D());
			//Color3f phase_sampled = Color3f(pRec_samp.pdf);
			//medium_throughput *= phaseVal;
			//BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d));
			//Color3f bsdf_sampled = its.mesh->getBSDF()->sample(bsdfRecord, sampler->next2D());
			//Ray3f newRay(its.p, its.toWorld(bsdfRecord.wo));
			Vector3f wo = Frame(-pRec_samp.wi).toWorld(pRec_samp.wo);
            Ray3f newRay = Ray3f(mRec.p, wo);

			float pdf_em_phaseDir = 0;
			if (sampler->next1D() < P_NO_ABSORTION) {
				Color3f Li = Li_rec(scene, sampler, newRay, pdf_em_phaseDir) / P_NO_ABSORTION;

				float w_bsdf;
				// if (bsdfRecord.measure == EDiscrete) {
				// 	w_bsdf = 1.0;
				// } else {
				w_bsdf = pRec_samp.pdf;
					//w_bsdf = its.mesh->getBSDF()->pdf(bsdfRecord);
				//}
				if (w_bsdf > Epsilon) {	 // Avoiding -nans
					w_bsdf /= w_bsdf + (pdflight * pdf_em_phaseDir);
				}
				radiance += medium_throughput * Li * phase_sampled * w_bsdf;
				//radiance += Li * bsdf_sampled * w_bsdf;
			}

		// ---------------------- Next event estimation -----------------------
		//if (bsdfRecord.measure != EDiscrete) {
			EmitterQueryRecord emitterRecord(its.p);
			Color3f Le_emitter = em->sample(emitterRecord, sampler->next2D(), 0.);
			
			PhaseFunctionQueryRecord pRec(its.toLocal(-ray.d),its.toLocal(emitterRecord.wi), ESolidAngle); //to_local??
			float phaseVal = phase->eval(pRec);
			float pdf_phase_emDir = pRec.pdf;
			float cos_theta_i = its.shFrame.n.dot(emitterRecord.wi);

			// BSDFQueryRecord bsdfRecord_eval(its.toLocal(-ray.d),
			// 					its.toLocal(emitterRecord.wi), its.uv, ESolidAngle);
			// auto throughput = its.mesh->getBSDF()->eval(bsdfRecord_eval);
			// float pdf_bsdf_emDir = its.mesh->getBSDF()->pdf(bsdfRecord_eval);

			// Visibility query
			Ray3f shadowRay(its.p, emitterRecord.wi);
			Intersection its_shadow;
			bool intersect_shadow = scene->rayIntersect(shadowRay, its_shadow);
			if (!intersect_shadow || its_shadow.t > (emitterRecord.dist - Epsilon)) {
				float w_em = pdflight * em->pdf(emitterRecord);
				if (w_em > Epsilon) {	 // Avoiding -nans
					w_em /= w_em + pdf_phase_emDir;
				}
				
				// radiance += w_em * Le_emitter * its.shFrame.n.dot(emitterRecord.wi) * throughput
				// 	  		/ pdflight;
				radiance += medium_throughput * w_em * Le_emitter * cos_theta_i * phaseVal
							/ pdflight;
			}
		} else {
			if(its.mesh->hasMedium()){ //uncomment if the medium is on a mesh
                medium_throughput *= mRec.transmittance / mRec.pdfFailure;
			}
			
			if (its.mesh->isEmitter()) {	// Account for visible light sources
				EmitterQueryRecord emR;
				emR.wi = ray.d;
				emR.n = its.shFrame.n;
				emR.dist = its.t;
				emR.p = its.p;

				pdf_em_back = its.mesh->getEmitter()->pdf(emR);
				return medium_throughput * its.mesh->getEmitter()->eval(emR);
			}	
			pdf_em_back = 0;

			// NEE, but needed for BSDF sampling
			float pdflight;
			const Emitter* em = scene->sampleEmitter(sampler->next1D(), pdflight);

			// -------------- Generating next step with BSDF sampling --------------
			// PhaseFunctionQueryRecord pRec_samp(its.toLocal(-ray.d), Vector3f(0.0f), ESolidAngle); //to_local??
			// float phaseVal = phase->sample(pRec_samp, sampler->next2D());
			// Color3f phase_sampled = Color3f(pRec_samp.pdf);
			//medium_throughput *= phaseVal;
			BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d));
			Color3f bsdf_sampled = its.mesh->getBSDF()->sample(bsdfRecord, sampler->next2D());
			Ray3f newRay(its.p, its.toWorld(bsdfRecord.wo));
			// Vector3f wo = Frame(-pRec_samp.wi).toWorld(pRec_samp.wo);
            // Ray3f newRay = Ray3f(mRec.p, wo);

			float pdf_em_bsdfDir = 0;
			if (sampler->next1D() < P_NO_ABSORTION) {
				Color3f Li = Li_rec(scene, sampler, newRay, pdf_em_bsdfDir) / P_NO_ABSORTION;

				float w_bsdf;
				if (bsdfRecord.measure == EDiscrete) {
					w_bsdf = 1.0;
				} else {
				    //w_bsdf = pRec_samp.pdf;
					w_bsdf = its.mesh->getBSDF()->pdf(bsdfRecord);
				}
				if (w_bsdf > Epsilon) {	 // Avoiding -nans
					w_bsdf /= w_bsdf + (pdflight * pdf_em_bsdfDir);
				}
				//radiance += medium_throughput * Li * phaseVal-> * w_bsdf;
				radiance += medium_throughput * Li * bsdf_sampled * w_bsdf;
			}

		// ---------------------- Next event estimation -----------------------
			if (bsdfRecord.measure != EDiscrete) {
				EmitterQueryRecord emitterRecord(its.p);
				Color3f Le_emitter = em->sample(emitterRecord, sampler->next2D(), 0.);
				
				// PhaseFunctionQueryRecord pRec(its.toLocal(-ray.d),its.toLocal(emitterRecord.wi), ESolidAngle); //to_local??
				// float phaseVal = phase->eval(pRec);
				// float pdf_phase_emDir = pRec.pdf;
				// float cos_theta_i = its.shFrame.n.dot(emitterRecord.wi);

				BSDFQueryRecord bsdfRecord_eval(its.toLocal(-ray.d),
									its.toLocal(emitterRecord.wi), its.uv, ESolidAngle);
				auto throughput = its.mesh->getBSDF()->eval(bsdfRecord_eval);
				float pdf_bsdf_emDir = its.mesh->getBSDF()->pdf(bsdfRecord_eval);

				// Visibility query
				Ray3f shadowRay(its.p, emitterRecord.wi);
				Intersection its_shadow;
				bool intersect_shadow = scene->rayIntersect(shadowRay, its_shadow);
				if (!intersect_shadow || its_shadow.t > (emitterRecord.dist - Epsilon)) {
					float w_em = pdflight * em->pdf(emitterRecord);
					if (w_em > Epsilon) {	 // Avoiding -nans
						w_em /= w_em + pdf_bsdf_emDir;
					}
					
					radiance += medium_throughput * w_em * Le_emitter * its.shFrame.n.dot(emitterRecord.wi) * throughput
						  		/ pdflight;
					// radiance += medium_throughput * w_em * Le_emitter * cos_theta_i * phaseVal
					// 			/ pdflight;
				}
			}
		}

		return radiance;	
	}

public:
	PathTracingVOL(const PropertyList& props) {
		/* No parameters this time */
	}

	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
		float unused;
		return Li_rec(scene, sampler, ray, unused);
	}

	std::string toString() const {
		return "Path tracing VOL []";
	}
};

NORI_REGISTER_CLASS(PathTracingVOL, "path_vol") ;
NORI_NAMESPACE_END