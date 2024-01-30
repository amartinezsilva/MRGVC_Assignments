#pragma once

#include <nori/object.h>
#include <nori/bbox.h>
#include <nori/phase_function.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

class Medium;



struct MediumInterface {
    // Inside and outside medium
    Medium *inside, *outside;

    // When only one medium is provided, both sides are the same
    MediumInterface(): inside(NULL), outside(NULL) { }

    MediumInterface(Medium *medium)
        : inside(medium), outside(medium) { }
    
    MediumInterface(Medium *inside, Medium *outside)
        : inside(inside), outside(outside) { }

    bool IsMediumTransition() const { return inside != outside; }
};


struct MediumInteraction {
    
    /// Traveled unoccluded distance along the ray
    float t;

    /// Location of the scattering interaction
    Point3f p;

    // Incident direction
    Vector3f wi;
    // Scattered direction
    Vector3f wo;

    /// Phase function for the medium
    const PhaseFunction *phase;

    /// Pointer to the associated medium
    const Medium *medium;

    MediumInteraction() {medium = NULL; phase = NULL;};
    MediumInteraction(Point3f p_, float t_, Vector3f wi_):
    p(p_), t(t_), wi(wi_), phase(NULL), medium(NULL) { }

    MediumInteraction(Point3f p_, float t_, Vector3f wi_, const PhaseFunction *phase_, const Medium *medium_):
    p(p_), t(t_), wi(wi_), phase(phase_), medium(medium_) { }

    bool isValid() const { return phase != NULL; }
};

//homogeneous medium

class Medium : public NoriObject {
public:


    virtual Color3f Tr(const Ray3f &ray) const = 0;
    virtual Color3f sample(const Ray3f &ray, const Point2f &sample, MediumInteraction &mRec) const = 0;


    /// Return the phase function of this medium
    inline const PhaseFunction *getPhaseFunction() const { return m_phaseFunction; }

    // /// Determine whether the medium is homogeneous
    // virtual bool isHomogeneous() const = 0;

    // /// For homogeneous media: return the absorption coefficient
    // inline const Color3f &getSigmaA() const { return m_sigmaA; }

    // /// For homogeneous media: return the scattering coefficient
    // inline const Color3f &getSigmaS() const { return m_sigmaS; }

    // /// For homogeneous media: return the extinction coefficient
    // inline const Color3f &getSigmaT() const { return m_sigmaT; }

    // /// For homogeneous media: return the albedo coefficient
    // inline const Color3f get_albedo() const { return m_albedo; }

    /// Add a child
    /// Register a child object (e.g. a BSDF) with the mesh
    virtual void addChild(NoriObject *child, const std::string& name = "none") override
    {
        switch (child->getClassType()) {
        case EPhaseFunction:
            if (m_phaseFunction)
                throw NoriException(
                    "Medium: tried to register multiple instances!");
            m_phaseFunction = static_cast<PhaseFunction *>(child);
            break;

        default:
            throw NoriException("Medium::addChild(<%s>) is not supported!",
                                classTypeName(child->getClassType()));
        }
    }

    EClassType getClassType() const { return EMedium; }

protected:
     //Variables
    PhaseFunction* m_phaseFunction = NULL;
    // Color3f m_sigmaA;
    // Color3f m_sigmaS;
    // Color3f m_sigmaT;
    // Color3f m_albedo;
};


NORI_NAMESPACE_END