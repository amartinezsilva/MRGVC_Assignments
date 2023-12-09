#include <nori/PhaseFunction.h>

NORI_NAMESPACE_BEGIN



float PhaseFunction::sample_p(const Vector3f &wo, Vector3f &wi, const Point2f &sample) const {
    wi = Warp::squareToUniformSphere(sample);
    return INV_FOURPI;
}

std::string PhaseFunction::toString() const {
    return tfm::format("[ Isotropic Phase Function ]");
}


inline float PhaseHG(float cosTheta, float g) {
    float denom = 1 + g * g + 2 * g * cosTheta;
    return INV_FOURPI * (1 - g * g) / (denom * std::sqrt(denom));
}


HenyeyGreenstein::HenyeyGreenstein(float g) : m_g(g) { }

float HenyeyGreenstein::p(const Vector3f &wo, const Vector3f &wi) const {
    return PhaseHG(wo.dot(wi), m_g);
}

float HenyeyGreenstein::sample_p(const Vector3f &wo, Vector3f *wi, const Point2f &sample) const{
    //----------------------------// To fill in //---------------------------//

    return 0.0;
}



std::string HenyeyGreenstein::toString() const {
    return tfm::format("[ HenyeyGreenstein Phase Function -> g: %f ]", g);
}


NORI_REGISTER_CLASS(PhaseFunction, "isotropic");
NORI_REGISTER_CLASS(HenyeyGreenstein, "henyey_greenstein");


NORI_NAMESPACE_END