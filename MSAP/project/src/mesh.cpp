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


#include <nori/mesh.h>
#include <nori/bbox.h>
#include <nori/bsdf.h>
#include <nori/medium.h>
#include <nori/emitter.h>
#include <nori/warp.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

Mesh::Mesh() { }

Mesh::~Mesh() {
    m_pdf.clear();
    delete m_bsdf;
    delete m_emitter;
}

void Mesh::activate() {
    if (!m_bsdf) {
        /* If no material was assigned, instantiate a diffuse BRDF */
        m_bsdf = static_cast<BSDF *>(
            NoriObjectFactory::createInstance("diffuse", PropertyList()));
    }

    // m_pdf is already a DiscretePDF
    m_pdf.reserve(m_F.cols());

    for(uint32_t i = 0 ; i < m_F.cols() ; ++i) {
        m_pdf.append(surfaceArea(i));
        m_surfaceArea += surfaceArea(i);
    }

    m_pdf.normalize();
}

float Mesh::surfaceArea(n_UINT index) const {
    n_UINT i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);

    const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);

    return 0.5f * Vector3f((p1 - p0).cross(p2 - p0)).norm();
}

bool Mesh::rayIntersect(n_UINT index, const Ray3f &ray, float &u, float &v, float &t) const {
    n_UINT i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);
    const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);

    /* Find vectors for two edges sharing v[0] */
    Vector3f edge1 = p1 - p0, edge2 = p2 - p0;

    /* Begin calculating determinant - also used to calculate U parameter */
    Vector3f pvec = ray.d.cross(edge2);

    /* If determinant is near zero, ray lies in plane of triangle */
    float det = edge1.dot(pvec);

    if (det > -1e-8f && det < 1e-8f)
        return false;
    float inv_det = 1.0f / det;

    /* Calculate distance from v[0] to ray origin */
    Vector3f tvec = ray.o - p0;

    /* Calculate U parameter and test bounds */
    u = tvec.dot(pvec) * inv_det;
    if (u < 0.0 || u > 1.0)
        return false;

    /* Prepare to test V parameter */
    Vector3f qvec = tvec.cross(edge1);

    /* Calculate V parameter and test bounds */
    v = ray.d.dot(qvec) * inv_det;
    if (v < 0.0 || u + v > 1.0)
        return false;

    /* Ray intersects triangle -> compute t */
    t = edge2.dot(qvec) * inv_det;

    return t >= ray.mint && t <= ray.maxt;
}

void Mesh::setHitInformation(uint32_t index, const Ray3f &ray, Intersection & its) const {
    /* Find the barycentric coordinates */
    Vector3f bary;
    bary << 1-its.uv.sum(), its.uv;

    /* Vertex indices of the triangle */
    uint32_t idx0 = m_F(0, index), idx1 = m_F(1, index), idx2 = m_F(2, index);

    Point3f p0 = m_V.col(idx0), p1 = m_V.col(idx1), p2 = m_V.col(idx2);

    /* Compute the intersection positon accurately
       using barycentric coordinates */
    its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;

    /* Compute proper texture coordinates if provided by the mesh */
    if (m_UV.size() > 0)
        its.uv = bary.x() * m_UV.col(idx0) +
                 bary.y() * m_UV.col(idx1) +
                 bary.z() * m_UV.col(idx2);

    /* Compute the geometry frame */
    its.geoFrame = Frame((p1-p0).cross(p2-p0).normalized());

    if (m_N.size() > 0) {
        /* Compute the shading frame. Note that for simplicity,
           the current implementation doesn't attempt to provide
           tangents that are continuous across the surface. That
           means that this code will need to be modified to be able
           use anisotropic BRDFs, which need tangent continuity */

        its.shFrame = Frame(
                (bary.x() * m_N.col(idx0) +
                 bary.y() * m_N.col(idx1) +
                 bary.z() * m_N.col(idx2)).normalized());
    } else {
        its.shFrame = its.geoFrame;
    }
}

BoundingBox3f Mesh::getBoundingBox(n_UINT index) const {
    BoundingBox3f result(m_V.col(m_F(0, index)));
    result.expandBy(m_V.col(m_F(1, index)));
    result.expandBy(m_V.col(m_F(2, index)));
    return result;
}

Point3f Mesh::getCentroid(n_UINT index) const {
    return (1.0f / 3.0f) *
        (m_V.col(m_F(0, index)) +
         m_V.col(m_F(1, index)) +
         m_V.col(m_F(2, index)));
}

/**
 * \brief Uniformly sample a position on the mesh with
 * respect to surface area. Returns both position and normal
 */
void Mesh::samplePosition(const Point2f &sample, EmitterQueryRecord & lRec) const
{
	Point2f new_sample = sample;

    size_t triIndex = m_pdf.sampleReuse(new_sample(0));

    uint32_t i0 = m_F(0, triIndex), i1 = m_F(1, triIndex), i2 = m_F(2, triIndex);

    //sample position on triangule
    Point2f barycentric_coords = Warp::squareToUniformTriangle(new_sample);
    
	float alpha = barycentric_coords[0];
    float beta = barycentric_coords[1];
    float gamma = 1.0f - alpha - beta;

    // Compute the sampled position
    lRec.p = alpha * m_V.col(i0) + beta * m_V.col(i1) + gamma * m_V.col(i2);

    // Compute the sampled normal
    if (m_N.size() > 0) {
        lRec.n = alpha * m_N.col(i0) + beta * m_N.col(i1) + gamma * m_N.col(i2).normalized();
    } else {
        Point3f p0 = m_V.col(i0);
        Point3f p1 = m_V.col(i1);
        Point3f p2 = m_V.col(i2);
        lRec.n = (p1-p0).cross(p2-p0).normalized();
    }
    // Compute the sampled uv coordinates
    if (m_UV.size() > 0) {
        lRec.uv = alpha * m_UV.col(i0) + beta * m_UV.col(i1) + gamma * m_UV.col(i2);	
    }

    return;	
}

/// Return the surface area of the given triangle
float Mesh::pdf(const Point3f &p) const
{
    float meshArea = m_pdf.getNormalization();
	
	return meshArea;
}


void Mesh::addChild(NoriObject *obj, const std::string& name) {
    switch (obj->getClassType()) {
        case EBSDF:
            if (m_bsdf)
                throw NoriException(
                    "Mesh: tried to register multiple BSDF instances!");
            m_bsdf = static_cast<BSDF *>(obj);
            break;

        case EEmitter: {
                Emitter *emitter = static_cast<Emitter *>(obj);
                if (m_emitter)
                    throw NoriException(
                        "Mesh: tried to register multiple Emitter instances!");
                m_emitter = emitter;
            }
            break;

        case EMedium:
            if (!m_mediumInterface.inside)
                m_mediumInterface.inside = static_cast<Medium *>(obj);
            else if (!m_mediumInterface.outside)
                m_mediumInterface.outside = static_cast<Medium *>(obj);
            break;

        default:
            throw NoriException("Mesh::addChild(<%s>) is not supported!",
                                classTypeName(obj->getClassType()));
    }
}

std::string Mesh::toString() const {
    return tfm::format(
        "Mesh[\n"
        "  name = \"%s\",\n"
        "  vertexCount = %i,\n"
        "  triangleCount = %i,\n"
        "  bsdf = %s,\n"
        "  emitter = %s\n"
        "  inside = %s,\n"
        "  outside = %s,\n"
        "]",
        m_name,
        m_V.cols(),
        m_F.cols(),
        m_bsdf ? indent(m_bsdf->toString()) : std::string("null"),
        m_emitter ? indent(m_emitter->toString()) : std::string("null"),
        m_mediumInterface.inside ? indent(m_mediumInterface.inside->toString()) : std::string("null"),
        m_mediumInterface.outside ? indent(m_mediumInterface.outside->toString()) : std::string("null")
    );
}

std::string Intersection::toString() const {
    if (!mesh)
        return "Intersection[invalid]";

    return tfm::format(
        "Intersection[\n"
        "  p = %s,\n"
        "  t = %f,\n"
        "  uv = %s,\n"
        "  shFrame = %s,\n"
        "  geoFrame = %s,\n"
        "  mesh = %s\n"
        "]",
        p.toString(),
        t,
        uv.toString(),
        indent(shFrame.toString()),
        indent(geoFrame.toString()),
        mesh ? mesh->toString() : std::string("null")
    );
}

NORI_NAMESPACE_END