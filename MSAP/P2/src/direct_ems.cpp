#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class DirectEmitterSamplingIntegrator: public Integrator {
public:
	DirectEmitterSamplingIntegrator(const PropertyList& props) {
		/* No parameters this time */
	}

	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {

		Color3f Lo(0.);

		// Find the surface that is visible in the requested direction 
		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return scene->getBackground(ray);

		EmitterQueryRecord emitterRecord(its.p);

		float pdflight;
		// Sample random light source
		const Emitter *random_emitter = scene->sampleEmitter(sampler->next1D(), pdflight);

		Color3f Le = random_emitter->sample(emitterRecord, sampler->next2D(), sampler->next1D());

		float pdfpositionlight = emitterRecord.pdf;

		// Finally, we evaluate the BSDF. For that, we need to build
		// a BSDFQueryRecord from the outgoing direction (the direction
		// of the primary ray, in ray.d), and the incoming direction 
		// (the direction to the light source, in emitterRecord.wi). 
		// Note that: a) the BSDF assumes directions in the local frame
		// of reference; and b) that both the incoming and outgoing 
		// directions are assumed to start from the intersection point. 	
		BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d), 
							its.toLocal(emitterRecord.wi), its.uv, ESolidAngle);

		// For each light, we accomulate the incident light times the 
		// foreshortening times the BSDF term (i.e. the render equation). 
		Lo += Le * its.shFrame.n.dot(emitterRecord.wi) * 
							its.mesh->getBSDF()->eval(bsdfRecord);
		
		Lo = Lo / (pdflight*pdfpositionlight);

		// Check if intersected material is emitter
		if(its.mesh->isEmitter()){
			EmitterQueryRecord emitterRecordMaterial;
			Lo += its.mesh->getEmitter()->eval(emitterRecordMaterial);
		}

		return Lo;
	}

	std::string toString() const {
		return "Direct Emitter Sampling Integrator []";
	}
};

NORI_REGISTER_CLASS(DirectEmitterSamplingIntegrator, "direct_ems");
NORI_NAMESPACE_END
