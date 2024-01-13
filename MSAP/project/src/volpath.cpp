/*
Author: Jingyu Wang, ETHz Data Science Master student
*/
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>
#include <nori/medium.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class VolPath : public Integrator {
public:
    VolPath(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Direct_MIS(const Scene *scene, Sampler *sampler, const Intersection& its, const MediumInteraction& mRec) const
    {
        float lightPdf = 0.f, scatteringPdf = 0.f;
        Color3f L(0.f);
        Color3f beta(0.f);
        Intersection its_2;

        /* Emiiter sampling */
		EmitterQueryRecord lRec(its.p);
		const Emitter *random_emitter = scene->sampleEmitter(sampler->next1D(), lightPdf);
		Color3f Le = random_emitter->sample(lRec, sampler->next2D(), 0.);
        if (its.isSurfaceIteraction) // surface interaction
        {
            BSDFQueryRecord bRec(its.toLocal(mRec.wi), its.toLocal(lRec.wi), its.uv, its.mesh->getBSDF()->isDelta()? EDiscrete :ESolidAngle);
            beta = its.mesh->getBSDF()->eval(bRec) * std::max(lRec.wi.dot(its.shFrame.n), 0.f);
            scatteringPdf = its.mesh->getBSDF()->pdf(bRec);

            Color3f Tr(0.f);
            Ray3f shadowRay(lRec.ref, lRec.wi, Epsilon, (lRec.ref - lRec.p).norm()-Epsilon, mRec.medium);
            if (shadowRay.medium)
                Tr = shadowRay.medium->Tr(shadowRay);
            else
                Tr = Color3f(1.f);
            if (!scene->rayIntersect(shadowRay, its_2))
            {
                if (random_emitter->isDelta())
                    L += Tr * beta * Le;
                else if (random_emitter->isInfinity())
                {
                    if (!mRec.medium && lightPdf + scatteringPdf > 1e-9)
                        L += beta * lightPdf / (lightPdf + scatteringPdf) * Le;
                }
                else if (lightPdf + scatteringPdf > 1e-9)
                    L += Tr * beta * lightPdf / (lightPdf + scatteringPdf) * Le;
            }
        }
        else // medium interaction
        {
            PhaseQueryRecord pRec(mRec.wi, lRec.wi);
            scatteringPdf = mRec.phase->pdf(pRec);
            beta = Color3f(scatteringPdf);
            beta = Color3f(1.f);
            Color3f Tr(0.f);
            Ray3f shadowRay(lRec.ref, lRec.wi, Epsilon, (lRec.ref - lRec.p).norm()-Epsilon, mRec.medium);
            if (shadowRay.medium)
                Tr = shadowRay.medium->Tr(shadowRay);
            else
                Tr = Color3f(1.f);
            if (!scene->rayIntersect(shadowRay, its_2))
            {
                if (random_emitter->isDelta())
                    L += Tr * beta * Le;
                else if (lightPdf + scatteringPdf > 1e-9 && !random_emitter->isInfinity())
                    L += beta * lightPdf / (lightPdf + scatteringPdf) * Le;
            }
        }

        /* Sacttering sampling*/
        Ray3f shadowRay;
        bool sampleSpecular = false;
        if (its.isSurfaceIteraction) // surface interaction
        {
            BSDFQueryRecord bRec(its.toLocal(mRec.wi));
            beta = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
            scatteringPdf = its.mesh->getBSDF()->pdf(bRec);
            if (Frame::cosTheta(bRec.wo) > 0)
                shadowRay = Ray3f(its.p, its.toWorld(bRec.wo), Epsilon, INFINITY, its.mesh->getMediumInterface().outside);
            else
                shadowRay = Ray3f(its.p, its.toWorld(bRec.wo), Epsilon, INFINITY, its.mesh->getMediumInterface().inside);
            if(scene->rayIntersect(shadowRay, its_2))
            {
                if (its_2.mesh->isEmitter())
                {
                    shadowRay.maxt = (its_2.p - shadowRay.o).norm();
                    Color3f Tr(0.f);
                    Ray3f shadowRayEmitter(lRec.ref, lRec.wi, Epsilon, (lRec.ref - lRec.p).norm()-Epsilon, mRec.medium);
                    if (shadowRayEmitter.medium)
                        Tr = shadowRay.medium->Tr(shadowRayEmitter);  
                    else
                        Tr = Color3f(1.f);
                    EmitterQueryRecord lRec(mRec.p);
                    lRec.wi = shadowRay.d;
				    lRec.n = its_2.shFrame.n;
				    lRec.dist = its_2.t;
				    lRec.p = its_2.p;

                    lightPdf = its_2.mesh->getEmitter()->pdf(lRec);
                    if (lightPdf + scatteringPdf > 1e-9)
                    {
                        L += Tr * beta * scatteringPdf / (lightPdf + scatteringPdf) * its_2.mesh->getEmitter()->eval(lRec);
                    }
                }
            }
            else // env map
            {
                if (!mRec.medium)
                {
                    EmitterQueryRecord lRec(mRec.p);
                    lRec.wi = its.toWorld(bRec.wo);
                    
                    for (auto emitter: scene->getLights())
                    {
                        if (emitter->isInfinity())
                        {
                            lightPdf = emitter->pdf(lRec);
                            if (lightPdf + scatteringPdf > 1e-9)
                            {
                                L += beta * scatteringPdf / (lightPdf + scatteringPdf) * emitter->eval(lRec);
                            }
                        }
                    }
                }
            }
        }
        else // medium interaction
        {
            PhaseQueryRecord pRec(mRec.wi);
            scatteringPdf = mRec.phase->sample(pRec, sampler->next2D());
            beta = Color3f(scatteringPdf);
            // cout << beta << endl;
            shadowRay = Ray3f(mRec.p, pRec.wo, Epsilon, INFINITY, mRec.medium);
            if(scene->rayIntersect(shadowRay, its_2))
            {
                if (its_2.mesh->isEmitter())
                {
                    shadowRay.maxt = (its_2.p - shadowRay.o).norm();
                    Color3f Tr(0.f);
                    Ray3f shadowRayEmitter(lRec.ref, lRec.wi, Epsilon, (lRec.ref - lRec.p).norm()-Epsilon, mRec.medium);
                    if (shadowRayEmitter.medium)
                        Tr = shadowRay.medium->Tr(shadowRayEmitter);  
                    else
                        Tr = Color3f(1.f);
                    EmitterQueryRecord pRec(mRec.p);
                    pRec.wi = shadowRay.d;
				    pRec.n = its_2.shFrame.n;
				    pRec.dist = its_2.t;
				    pRec.p = its_2.p;
                    if (lightPdf + scatteringPdf > 1e-9)
                    {
                        L += beta * scatteringPdf / (lightPdf + scatteringPdf) * its_2.mesh->getEmitter()->eval(pRec);
                    }
                }
            }
        }

        return L;
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const{
        Color3f L(0.0f), beta(1.f);
		Ray3f ray_s = ray; // Next ray
        bool bounceSpecular = false;

        Intersection its;
        if (!scene->rayIntersect(ray, its))
			return scene->getBackground(ray);

		for(int bounces = 0; bounces < 100 ; bounces++) 
        {
            // Find intersection
            Intersection its;
            bool findIntersection = scene->rayIntersect(ray_s, its);

            float tmax;
			if(findIntersection)
				tmax = (its.p - ray_s.o).norm();
			else
				tmax =its.t;
            ray_s.maxt = tmax;

            // Sample the participating medim, if present
            MediumInteraction mRec;
            if (ray_s.medium)
            {
                beta *= ray_s.medium->sample(ray_s, sampler->next2D(), mRec);
            }
            else
            {
                mRec.wi = -ray_s.d;
                mRec.wi.normalize();
                mRec.p = its.p;
            }
            
            if (mRec.isValid()) // Medium interaction
            {
                its.isSurfaceIteraction = false;
                L += beta * Direct_MIS(scene, sampler, its, mRec);
                // Sample next ray
                PhaseQueryRecord pRec((mRec.wi).normalized());
                mRec.phase->sample(pRec, sampler->next2D());
                ray_s = Ray3f(mRec.p, pRec.wo, Epsilon, INFINITY, ray_s.medium);
            }
            else // Surface interaction
            {
                its.isSurfaceIteraction = true;
                if (bounces == 0 || bounceSpecular) // emitted light at path vertex or from the env 
                {
                    if (findIntersection)
                    {
                        if (its.mesh->isEmitter()) // Intersection with emitter
                        {
                            auto emitter = its.mesh->getEmitter();
                            EmitterQueryRecord lRec(ray_s.o);
                            lRec.wi = ray_s.d;
                            lRec.n = its.shFrame.n;
                            lRec.dist = its.t;
                            lRec.p = its.p;
                            Color3f Le = emitter->eval(lRec);
                            if (bounces == 0 || bounceSpecular) // First bounce += Le || specular += beta * Le  
                                L += beta * Le;
                        }  
                    }
                    else if(!ray.medium) // env map
                    {
                        for (auto emitter: scene->getLights())
                        {
                            if (emitter->isInfinity())
                            {
                                EmitterQueryRecord lRec(its.p);
                                lRec.wi = ray_s.d;
                                if (bounces == 0 || bounceSpecular) 
                                    L += beta * emitter->eval(lRec);
                            }
                        }
                        break;
                    }
                }

                if (!findIntersection)
                    break;

                L += beta * Direct_MIS(scene, sampler, its, mRec);

                // Sample bsdf to get new direction
                auto bsdf = its.mesh->getBSDF();
                BSDFQueryRecord bRec(its.toLocal(-ray_s.d));
                bRec.uv = its.uv;
                const Color3f brdf = bsdf->sample(bRec, sampler->next2D());
                if (bRec.measure == EDiscrete)
                    bounceSpecular = true;
                else
                    bounceSpecular = false;

                beta *= brdf;
                if (Frame::cosTheta(bRec.wo) < 0)
                    ray_s = Ray3f(its.p, (its.toWorld(bRec.wo)).normalized(), Epsilon, INFINITY, its.mesh->getMediumInterface().inside);
                else
                    ray_s = Ray3f(its.p, (its.toWorld(bRec.wo)).normalized(), Epsilon, INFINITY, its.mesh->getMediumInterface().outside);
                    
            }

            // Russian roulette termination starts from the 4th iteration
            if (bounces > 4)
            {
                const float q = std::min(beta.maxCoeff(), 0.99f);
                if (sampler->next1D() > q || q == 0.f)
                    break;
                else
                    beta /= q;
            }
		}
        
		return L;
    }

    std::string toString() const {
        return "VolPath[]";
    }
};

NORI_REGISTER_CLASS(VolPath, "volpath");
NORI_NAMESPACE_END