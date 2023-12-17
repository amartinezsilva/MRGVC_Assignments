#pragma once

#include <nori/object.h>
#include <nori/bbox.h>
#include <nori/phaseFunction.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

class Medium;

struct MediumSamplingRecord {
    /// Traveled distance
    float t;

    /// Location of the scattering interaction
    Point3f p;

    /// Local particle orientation at \ref p
    Vector3f orientation;

    /**
     * \brief Specifies the transmittance along the segment [mint, t]
     *
     * When sampling a distance fails, this contains the
     * transmittance along the whole ray segment [mint, maxDist].
     */
    Color3f transmittance;

    /// The medium's absorption coefficient at \ref p
    Color3f sigmaA;

    /// The medium's scattering coefficient at \ref p
    Color3f sigmaS;

    /// Records the probability density of sampling a medium interaction at p
    float pdfSuccess;


    /**
     * When the \ref Medium::sampleDistance() is successful, this function
     * returns the probability of \a not having generated a medium interaction
     * until \ref t. Otherwise, it records the probability of
     * not generating any interactions in the whole interval [mint, maxt].
     * This probability is assumed to be symmetric with respect to
     * sampling from the other direction, which is why there is no
     * \c pdfFailureRev field.
     */
    float pdfFailure;

    /// Pointer to the associated medium
    const Medium *medium;

    inline MediumSamplingRecord() : medium(NULL) { }

    /// Return a pointer to the phase function
    inline const PhaseFunction *getPhaseFunction() const;

    /// Return a string representation
    std::string toString() const;
};

//homogeneous medium

class Medium : public NoriObject {
public:

    /**
     * \brief Compute the 1D density of sampling distance \a ray.maxt
     * along the ray using the sampling strategy implemented by
     * \a sampleDistance.
     *
     * The function computes the continuous densities in the case of
     * a successful \ref sampleDistance() invocation (in both directions),
     * as well as the Dirac delta density associated with a failure.
     * For convenience, it also stores the transmittance along the
     * supplied ray segment within \a mRec.
     */
    virtual void eval(const Ray3f &ray,
    MediumSamplingRecord &mRec) const = 0;

    /**
     * \brief Compute the transmittance along a ray segment
     *
     * Computes the transmittance along a ray segment
     * [mint, maxt] associated with the ray. It is assumed
     * that the ray has a normalized direction value.
     */
    virtual Color3f evalTransmittance(const Ray3f &ray, const Point2f &sample) const = 0;


    /// Return the phase function of this medium
    inline const PhaseFunction *getPhaseFunction() const { return m_phaseFunction; }

    /// Determine whether the medium is homogeneous
    virtual bool isHomogeneous() const = 0;

    /// For homogeneous media: return the absorption coefficient
    inline const Color3f &getSigmaA() const { return m_sigmaA; }

    /// For homogeneous media: return the scattering coefficient
    inline const Color3f &getSigmaS() const { return m_sigmaS; }

    /// For homogeneous media: return the extinction coefficient
    inline const Color3f &getSigmaT() const { return m_sigmaT; }

    /// For homogeneous media: return the albedo coefficient
    inline const Color3f get_albedo() const { return m_albedo; }

    /// Add a child
    /// Register a child object (e.g. a BSDF) with the mesh
    virtual void addChild(NoriObject *child, const std::string& name = "none") {};

    /// Return a string representation
    virtual std::string toString() const = 0;

    EClassType getClassType() const { return EMedium; }

protected:
     //Variables
    PhaseFunction* m_phaseFunction = NULL;
    Color3f m_sigmaA;
    Color3f m_sigmaS;
    Color3f m_sigmaT;
    Color3f m_albedo;
};

// class HomogeneousMedium : public Medium {
// public:
//         HomogeneousMedium(const PropertyList &props);
//         //HomogeneousMedium(PropertyList &props);
//         Color3f Tr(const Point3f &a, const Point3f &b) const;
//         Color3f evalTransmittance(const Ray3f &ray, const Point2f &sample) const;
//         Color3f sample(const Ray3f &ray, Sampler *sampler, MediumInteractionQuery &mi) const;

//         std::string toString() const override;
//         EClassType  getClassType() const override { return EMedium; }

// private:
//        const Color3f m_sigma_a, m_sigma_s, m_sigma_t;
//        const float m_g;
//        PhaseFunction * m_phaseFunction = nullptr;

// };



NORI_NAMESPACE_END