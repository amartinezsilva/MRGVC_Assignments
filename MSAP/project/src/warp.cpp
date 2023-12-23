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
    float r = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    return (std::abs(r - 1.0f) < Epsilon) ? INV_FOURPI : 0.0f;
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
    return (v[2] >= 0.0) && (std::abs(r - 1.0f) < Epsilon) ? INV_TWOPI : 0.0f;

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
    return (v[2] >= 0.0) && (r <= 1) ? std::cos(theta)*INV_PI : 0.0f;;
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    Vector3f warped_sample;
    float theta = atan(sqrt(-pow(alpha, 2) * log(1-sample.x())));
    float phi = 2 * M_PI * sample.y();
    warped_sample[0] = sin(theta) * cos(phi);
    warped_sample[1] = sin(theta) * sin(phi);
    warped_sample[2] = cos(theta);

    return warped_sample;
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    float r = sqrt(m[0]*m[0] + m[1]*m[1] + m[2]*m[2]);
    float theta = acos(m[2]);
    float beckmann = exp(-pow(tan(theta), 2)/ pow(alpha, 2));
    float beckmann_normalization = M_PI * pow(alpha, 2) * pow(cos(theta), 3);
    return (std::abs(r - 1.0f) < Epsilon && m[2] >= 0) ? (beckmann / beckmann_normalization) : 0.0f;
}

Vector3f Warp::squareToHenyeyGreenstein(const Point2f &sample, float g) {

    //use the inverse methode
    if(abs(g) <= Epsilon){
        return Warp::squareToUniformSphere(sample);
        //return 1.0 - 2*sample(0);
    }

    float scale = 1.0f / (2.0f * g);  //negative?
    float fraction = (1.0f - g*g) / (1.0f - g + 2.0f * g * std::max(sample(0), 0.001f));
    float costheta =  scale * (1.0f +  g * g - fraction * fraction);


    float phi = 2.0 * M_PI * sample(1);

    float theta = acos(costheta);

    return Vector3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
}

float Warp::squareToHenyeyGreensteinPDF(const Vector3f &p, float g){
    float cosTheta = p(2);
    float sqrtFrac = (1.0f + g * g - 2.0f * g * cosTheta);
    if(sqrtFrac <= 0.0f)
        return 0.0f;
    float pdf =  ((1.0f - g * g) / (4.0f * M_PI * pow(sqrtFrac, 1.5f)));

    return pdf;
}

float Warp::squareToHenyeyGreensteinPDF(const Vector3f &wo, const Vector3f &wi, float g){
    float cosTheta = wo.dot(wi);
    float sqrtFrac = (1.0f + g * g - 2.0f * g * cosTheta);
    if(sqrtFrac <= 0.0f)
        return 0.0f;
    float pdf =  ((1.0f - g * g) / (4.0f * M_PI * pow(sqrtFrac, 1.5f)));

    return pdf;
}


NORI_NAMESPACE_END