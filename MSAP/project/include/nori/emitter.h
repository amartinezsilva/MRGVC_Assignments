/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    v1 - Dec 2020
    Copyright (c) 2020 by Adrian Jarabo

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


#pragma once

#include <nori/object.h>
#include <nori/medium.h>


NORI_NAMESPACE_BEGIN


enum class EmitterType
{
	EMITTER_POINT,
	EMITTER_DISTANT_DISK,
	EMITTER_AREA,
	EMITTER_ENVIRONMENT,
	EMITTER_UNKNOWN
};


/**
 * \brief Data record for conveniently querying and sampling the
 * direct illumination technique implemented by a emitter
 */
struct EmitterQueryRecord {
    /// Pointer to the sampled emitter
    const Emitter* emitter;
    /// Origin point from which we sample the emitter
    Point3f ref;
    /// Sampled position on the light source
    Point3f p;
    /// Associated surface normal
    Normal3f n;
    /// Sampled texture coordinates on the light source
    Point2f uv;
    /// Solid angle density wrt. 'ref'
    float pdf;
    /// Direction vector from 'ref' to 'p'
    Vector3f wi;
    /// Distance between 'ref' and 'p'
    float dist;

    /// Create an unitialized query record
    EmitterQueryRecord() : emitter(nullptr) { }

    /// Create a new query record that can be used to sample a emitter
    EmitterQueryRecord(const Point3f& ref) : ref(ref) { }

    /**
     * \brief Create a query record that can be used to query the
     * sampling density after having intersected an area emitter
     */
    EmitterQueryRecord(const Emitter* emitter,
        const Point3f& ref, const Point3f& p,
        const Normal3f& n, const Point2f& uv) : emitter(emitter), ref(ref), p(p), n(n), uv(uv){
		wi = p - ref;
		dist = wi.norm();
		wi /= dist;
	}

	/// Return a human-readable string summary
	std::string toString() const;
};

struct VisibilityTester {
    Ray3f r;
    /**
     * @brief SetSegment the the ray
     * @param p1 the first point of the ray (origin)
     * @param eps1 a offset value for the first point
     * @param p2 the second point of the ray
     * @param eps2 a offset value for the second point
     */
    void SetSegment(const Point3f &p1, float eps1,
                    const Point3f &p2, float eps2 /*, float time*/) {
        //calculate the direction
        Vector3f d = (p2 - p1);

        //calculate the distance
        float dist = (p2 - p1).norm();

        //create the ray segment
        r = Ray3f(p1, d / dist, eps1 , dist * (1.f - eps2)); /*, time);*/
    }

    /**
     * @brief SetRay create a ray for the VisibilityTester
     * @param p the origin
     * @param w the direction (should be normalized)
     */
    void SetRay(const Point3f &p, const Vector3f &w/* , float eps, float time*/) {
        r = Ray3f(p, w);
    }
    /**
     * @brief Unoccluded check if the origin is visible
     * @param scene the current scene
     * @return true if the the point is not occluded
     */
    // bool Unoccluded(const Scene *scene) const {
    //     return !scene->rayIntersect(r);
    // }
};


/**
 * \brief Superclass of all emitters
 */
class Emitter : public NoriObject {
public:
    /**
     * \brief Sample the emitter and return the importance weight (i.e. the
     * value of the Emitter divided by the probability density
     * of the sample with respect to solid angles).
     *
     * \param lRec    An emitter query record (only ref is needed)
     * \param sample  A uniformly distributed sample on \f$[0,1]^2\f$
	 * \param u       Another optional sample that might be used in some scenarios.
     *
     * \return The emitter value divided by the probability density of the sample.
     *         A zero value means that sampling failed.
     */
    virtual Color3f sample(EmitterQueryRecord &lRec, const Point2f &sample, float optional_u) const = 0;

    /**
     * \brief Compute the probability of sampling \c lRec.p.
     *
     * This method provides access to the probability density that
     * is realized by the \ref sample() method.
     *
     * \param lRec
     *     A record with detailed information on the emitter query
     *
     * \return
     *     A probability/density value
     */
    virtual float pdf(const EmitterQueryRecord &lRec) const = 0;

    /**
     * \brief Evaluate the emitter
     *
     * \param lRec
     *     A record with detailed information on the emitter query
     * \return
     *     The emitter value, evaluated for each color channel
     */
    virtual Color3f eval(const EmitterQueryRecord &lRec) const = 0;

    /**
     * \brief Virtual destructor
     * */
    virtual ~Emitter() {}

    /**
     * \brief Return the type of object (i.e. Mesh/Emitter/etc.) 
     * provided by this instance
     * */
    virtual EClassType getClassType() const { return EEmitter; }

    /**
     * \brief Set the mesh if the emitter is attached to a mesh
     * */
    void setMesh(Mesh * mesh) { m_mesh = mesh; }

	EmitterType getEmitterType() const { return m_type; }

	bool isDelta() const { return m_type == EmitterType::EMITTER_POINT; }

    virtual void addChild(NoriObject *obj, const std::string& name = "none") override
    {
        switch (obj->getClassType()) {
            case EMedium:
                if (m_medium)
                    throw NoriException(
                        "Emitter: tried to register multiple BSDF instances!");
                m_medium = static_cast<Medium *>(obj);
                break;
            default:
                throw NoriException("Shape::addChild(<%s>) is not supported!",
                                    classTypeName(obj->getClassType()));
        }
    }


    const Medium *getMedium() const { return m_medium; }

protected:
    /// Pointer to the mesh if the emitter is attached to a mesh
    Mesh * m_mesh = nullptr;
	EmitterType m_type;
    Medium * m_medium = nullptr;
};

NORI_NAMESPACE_END
