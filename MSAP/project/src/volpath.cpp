#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <iostream>
#include <nori/sampler.h>
#include <nori/medium.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class VolPath : public Integrator {
public:
    VolPath(const PropertyList &props) {}

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const{
        Color3f Lo(0.0f), beta(1.f);
		Ray3f newRay = ray; // Next ray
        bool hitSpecular = false;

        Intersection its;
        if (!scene->rayIntersect(ray, its))
			return scene->getBackground(ray);

		for(int iterations = 0; iterations < 100 ; iterations++) {
            // Get intersection of new ray if exist
            Intersection its;
            bool getIntersection = scene->rayIntersect(newRay, its);

            float newRayMaxt;
			if(getIntersection)
				newRayMaxt = (its.p - newRay.o).norm();
			else
				newRayMaxt =its.t;
            newRay.maxt = newRayMaxt;

            // Sample the participating medim, if present
            MediumInteraction mRec;
            if (newRay.medium) {
                beta *= newRay.medium->sample(newRay, sampler->next2D(), mRec);
            }
            else {
                mRec.p = its.p;
                mRec.wi = -newRay.d;
                mRec.wi.normalize();  
            }
            
            if (mRec.isValid()) { // Medium interaction
                its.isSurfaceIteraction = false;
                Lo += beta * Direct_MIS(scene, sampler, its, mRec);
                // Sample next ray
                PhaseQueryRecord pRec((mRec.wi).normalized());
                mRec.phase->sample(pRec, sampler->next2D());
                newRay = Ray3f(mRec.p, pRec.wo, Epsilon, INFINITY, newRay.medium);
            }
            else { // Surface interaction
                its.isSurfaceIteraction = true;
                if (iterations == 0 || hitSpecular) { // emitted light at path vertex
                    if (getIntersection){
                        if (its.mesh->isEmitter()) { // Intersection with emitter
                            auto emitter = its.mesh->getEmitter();
                            EmitterQueryRecord lRec(newRay.o);
                            lRec.n = its.shFrame.n;
                            lRec.wi = newRay.d;
                            lRec.dist = its.t;
                            lRec.p = its.p;
                            Color3f Le = emitter->eval(lRec);
                            if (iterations == 0 || hitSpecular) // First bounce += Le || specular += beta * Le  
                                Lo += beta * Le;
                        }  
                    }
                }

                if (!getIntersection)
                    break;

                Lo += beta * Direct_MIS(scene, sampler, its, mRec);

                // Sample bsdf to find new direction of the ray
                auto bsdf = its.mesh->getBSDF();
                BSDFQueryRecord bRec(its.toLocal(-newRay.d));
                bRec.uv = its.uv;
                const Color3f brdf = bsdf->sample(bRec, sampler->next2D());
                if (bRec.measure == EDiscrete)
                    hitSpecular = true;
                else
                    hitSpecular = false;

                beta *= brdf;
                if (Frame::cosTheta(bRec.wo) < 0)
                    newRay = Ray3f(its.p, (its.toWorld(bRec.wo)).normalized(), Epsilon, INFINITY, its.mesh->getMediumInterface().inside);
                else
                    newRay = Ray3f(its.p, (its.toWorld(bRec.wo)).normalized(), Epsilon, INFINITY, its.mesh->getMediumInterface().outside);
                    
            }

            
            if (iterations > 4) { // Russian roulette 
                // it can end up since the 4th iteration
                const float probability = std::min(beta.maxCoeff(), 0.99f);
                if (sampler->next1D() > probability || probability == 0.f)
                    break;
                else
                    beta /= probability;
            }
		}
        
		return Lo;
    }

    Color3f Direct_MIS(const Scene *scene, Sampler *sampler, const Intersection& its, const MediumInteraction& mRec) const
    {
        float pdfLight = 0.f, pdfScattering = 0.f;
        Color3f Lo(0.f);
        Color3f beta(0.f);
        Intersection second_its;

        auto emitter = scene->getRandomEmitter(sampler->next1D());
        int number_of_emitters = scene->getLights().size();

        EmitterQueryRecord lRec(mRec.p);
        Color3f Le = emitter->sample(lRec, sampler->next2D(), 0.);
        pdfLight = emitter->pdf(lRec);

        //////////////////////
        // emitter sampling //
        //////////////////////
        if (its.isSurfaceIteraction) { // Interaction with one surface

            BSDFQueryRecord bRec(its.toLocal(mRec.wi), its.toLocal(lRec.wi), its.uv, its.mesh->getBSDF()->isDelta()? EDiscrete :ESolidAngle);
            beta = its.mesh->getBSDF()->eval(bRec) * std::max(lRec.wi.dot(its.shFrame.n), 0.f);
            
            pdfScattering = its.mesh->getBSDF()->pdf(bRec);

            Color3f Transmittance(0.f);
            if (lRec.shadowRay.medium)
                Transmittance = lRec.shadowRay.medium->Tr(lRec.shadowRay);
            else
                Transmittance = Color3f(1.f);
            if (!scene->rayIntersect(lRec.shadowRay, second_its))
            {
                if (emitter->isDelta())
                    Lo += number_of_emitters * Transmittance * beta * Le;
                else if (pdfLight + pdfScattering > 1e-9)
                    Lo += number_of_emitters * Transmittance * beta * pdfLight / (pdfLight + pdfScattering) * Le;
            }
        }
        else { // Interaction with medium

            PhaseQueryRecord pRec(mRec.wi, lRec.wi);
            pdfScattering = mRec.phase->pdf(pRec);
            // beta = Color3f(pdfScattering);
            beta = Color3f(1.f);
            Color3f Transmittance(0.f);
            if (lRec.shadowRay.medium)
                Transmittance = lRec.shadowRay.medium->Tr(lRec.shadowRay);
            else
                Transmittance = Color3f(1.f);
            if (!scene->rayIntersect(lRec.shadowRay, second_its))
            {
                if (emitter->isDelta())
                    Lo += number_of_emitters * Transmittance * beta * Le;
                else if (pdfLight + pdfScattering > 1e-9)
                    Lo += number_of_emitters * beta * pdfLight / (pdfLight + pdfScattering) * Le;
            }
        }

        /////////////////////////
        // Sacttering sampling //
        /////////////////////////
        Ray3f shadowRay;
        bool sampleSpecular = false;
        if (its.isSurfaceIteraction) { // Interaction with one surface
            BSDFQueryRecord bRec(its.toLocal(mRec.wi));
            beta = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
            pdfScattering = its.mesh->getBSDF()->pdf(bRec);
            if (Frame::cosTheta(bRec.wo) > 0)
                shadowRay = Ray3f(its.p, its.toWorld(bRec.wo), Epsilon, INFINITY, its.mesh->getMediumInterface().outside);
            else
                shadowRay = Ray3f(its.p, its.toWorld(bRec.wo), Epsilon, INFINITY, its.mesh->getMediumInterface().inside);

            if(scene->rayIntersect(shadowRay, second_its))
            {
                if (second_its.mesh->isEmitter())
                {
                    shadowRay.maxt = (second_its.p - shadowRay.o).norm();
                    Color3f Transmittance(0.f);
                    if (lRec.shadowRay.medium)
                        Transmittance = shadowRay.medium->Tr(lRec.shadowRay);  
                    else
                        Transmittance = Color3f(1.f);
                    EmitterQueryRecord lRec(mRec.p);
                    lRec.dist = second_its.t;
				    lRec.p = second_its.p;
				    lRec.n = second_its.shFrame.n;
                    pdfLight = second_its.mesh->getEmitter()->pdf(lRec);
                    if (pdfLight + pdfScattering > 1e-9)
                    {
                        Lo += number_of_emitters * Transmittance * beta * pdfScattering / (pdfLight + pdfScattering) * second_its.mesh->getEmitter()->eval(lRec);
                    }
                }
            }
        }
        else { // Interaction with medium
            PhaseQueryRecord pRec(mRec.wi);
            pdfScattering = mRec.phase->sample(pRec, sampler->next2D());
            beta = Color3f(pdfScattering);
            // cout << beta << endl;
            shadowRay = Ray3f(mRec.p, pRec.wo, Epsilon, INFINITY, mRec.medium);
            if(scene->rayIntersect(shadowRay, second_its))
            {
                if (second_its.mesh->isEmitter())
                {
                    shadowRay.maxt = (second_its.p - shadowRay.o).norm();
                    Color3f Transmittance(0.f);
                    if (lRec.shadowRay.medium)
                        Transmittance = shadowRay.medium->Tr(lRec.shadowRay);  
                    else
                        Transmittance = Color3f(1.f);
                    EmitterQueryRecord pRec(mRec.p);
				    pRec.n = second_its.shFrame.n;
				    pRec.dist = second_its.t;
				    pRec.p = second_its.p;
                    if (pdfLight + pdfScattering > 1e-9)
                    {
                        Lo += number_of_emitters * beta * pdfScattering / (pdfLight + pdfScattering) * second_its.mesh->getEmitter()->eval(pRec);
                    }
                }
            }
        }

        return Lo;
    }

    std::string toString() const {
        return "VolPath[]";
    }
};

NORI_REGISTER_CLASS(VolPath, "volpath");
NORI_NAMESPACE_END