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

#if !defined(__NORI_PHASEFUNCTION_H)
#define __NORI_PHASEFUNCTION_H

#include <nori/object.h>
#include <memory>

NORI_NAMESPACE_BEGIN

/**
 * \brief Convenience data structure used to pass multiple
 * parameters to the evaluation and sampling routines in \ref BSDF
 */
struct PhaseQueryRecord {
    /// Incident direction (in the local frame)
    Vector3f wi;

    /// Outgoing direction (in the local frame)
    Vector3f wo;

    /// Measure associated with the sample
    EMeasure measure;

    PhaseQueryRecord() { }
    /// Create a new record for sampling the phase function
    PhaseQueryRecord(const Vector3f &wi)
        : wi(wi){ }

    /// Create a new record for querying the phase function
    PhaseQueryRecord(const Vector3f &wi,
            const Vector3f &wo)
        : wi(wi), wo(wo) { }
};

class PhaseFunction : public NoriObject {
public:
    /// Release all memory
    virtual ~PhaseFunction() { }


    virtual float sample(PhaseQueryRecord &bRec, const Point2f &sample) const = 0;


    virtual float pdf(PhaseQueryRecord &bRec) const = 0;

    EClassType getClassType() const { return EPhaseFunction; }


};

NORI_NAMESPACE_END

#endif /* __NORI_SAMPLER_H */
