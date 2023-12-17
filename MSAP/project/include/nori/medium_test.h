/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#if !defined(__NORI_MEDIUM_H)
#define __NORI_MEDIUM_H

#include <nori/object.h>
#include <nori/phaseFunction.h>

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
/**
 * \brief Superclass of all bidirectional scattering distribution functions
 */
class Medium : public NoriObject {
public:
    /**
     * \brief Sample a distance along the ray segment [mint, maxt]
     *
     * Should ideally importance sample with respect to the transmittance.
     * It is assumed that the ray has a normalized direction value.
     *
     * \param ray      Ray, along which a distance should be sampled
     * \param mRec     Medium sampling record to be filled with the result
     * \return         \c false if the maximum distance was exceeded, or if
     *                 no interaction inside the medium could be sampled.
     */
    virtual bool sampleDistance(const Ray3f &ray,
        MediumSamplingRecord &mRec, const Point2f &sample) const = 0;

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

    /// Add a child
    virtual void addChild(NoriObject *obj) {}

    /// Return a string representation
    virtual std::string toString() const = 0;

    EClassType getClassType() const { return EMedium; }
protected:
     //Variabeles
    PhaseFunction* m_phaseFunction = NULL;
    Color3f m_sigmaA;
    Color3f m_sigmaS;
    Color3f m_sigmaT;

};


NORI_NAMESPACE_END

#endif