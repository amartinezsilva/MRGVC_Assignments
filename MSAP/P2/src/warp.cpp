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

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToTent(const Point2f &sample) {
    throw NoriException("Warp::squareToTent() is not yet implemented!");
}

float Warp::squareToTentPdf(const Point2f &p) {
    throw NoriException("Warp::squareToTentPdf() is not yet implemented!");
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    
    // Point2f warped_sample;
    // float r = std::sqrt(sample[0]);
    // float theta = 2 * M_PI * sample[1];
    // warped_sample[0] = r * std::cos(theta);
    // warped_sample[1] = r * std::sin(theta);
    // return warped_sample;

    // Map uniform random numbers to  [-1,1]
    Point2f uOffset = 2.f * sample - Vector2f(1, 1);
    // Handle degeneracy at the origin  
    if (uOffset[0] == 0 && uOffset[1] == 0)
        return Point2f(0.0, 0.0);
    // Apply concentric mapping to point
    float theta, r;
    if (std::abs(uOffset[0]) > std::abs(uOffset[1])) {
        r = uOffset[0];
        theta = M_PI/4.0 * (uOffset[1] / uOffset[0]);
    } else {
        r = uOffset[1];
        theta = M_PI/2.0 - M_PI/4.0 * (uOffset[0] / uOffset[1]);
    }

    return r * Point2f(std::cos(theta), std::sin(theta));
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    //return INV_PI;
    float r = sqrt(p[0]*p[0] + p[1]*p[1]);
    return (r <= 1.0) ? INV_PI : 0.0f;

}

Point2f Warp::squareToUniformTriangle(const Point2f& sample) {
    Point2f warped_sample = sample;
    if(sample[0] + sample[1] > 1) {
        warped_sample[0] = 1-sample[0];
        warped_sample[1] = 1-sample[1];
    }
    
    return warped_sample;
}

float Warp::squareToUniformTrianglePdf(const Point2f& p) {
    return ((p.array() >= 0).all() && (p.array() <= 1).all()) && (p[0] + p[1] <= 1) ? 2.0f : 0.0f;
}


Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    
    Vector3f warped_sample;

    float z = 1.0 - 2.0 * sample[0];
    float r = std::sqrt(std::max(0.0, 1.0 - z * z));
    float phi = 2.0 * M_PI * sample[1];

    warped_sample[0] =  r * std::cos(phi);
    warped_sample[1] =  r * std::sin(phi);
    warped_sample[2] = z;
    
    return warped_sample;
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    return INV_FOURPI;
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    
    Vector3f warped_sample;

    float z = sample[0];
    float r = std::sqrt(std::max(0.0, 1.0 - z * z));
    float phi = 2.0 * M_PI * sample[1];

    warped_sample[0] =  r * std::cos(phi);
    warped_sample[1] =  r * std::sin(phi);
    warped_sample[2] = z;
    
    return warped_sample;
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    float r = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    return (v[2] >= 0.0) && (r <= 1.0) ? INV_TWOPI : 0.0f;

}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    Vector3f warped_sample;
    Point2f d = Warp::squareToUniformDisk(sample);

    float z = std::sqrt(std::max(0.0, 1.0 - d[0] * d[0] - d[1] * d[1]));
    warped_sample[0] = d[0];
    warped_sample[1] = d[1];
    warped_sample[2] = z;

    return warped_sample;
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    float r = sqrt(v[0]*v[0] + v[1]*v[1]); 
    float theta = std::asin(r);
    return (v[2] >= 0.0) && (r <= 1.0) ? std::cos(theta)*INV_PI : 0.0f;;
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    throw NoriException("Warp::squareToBeckmann() is not yet implemented!");
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    throw NoriException("Warp::squareToBeckmannPdf() is not yet implemented!");
}

NORI_NAMESPACE_END