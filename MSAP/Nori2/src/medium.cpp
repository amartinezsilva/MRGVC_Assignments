#include <nori/medium.h>

NORI_NAMESPACE_BEGIN

Medium::Medium(const PropertyList &props) {
    m_sigma_a = (props.getColor("sigma_a", Color3f(0.2f)));
    m_sigma_s = (props.getColor("sigma_s", Color3f(0.2f)));
    m_sigma_t = m_sigma_a + m_sigma_s;
    m_albedo = m_sigma_s / m_sigma_t;
    // m_density_function = props.getInteger("density_function", 1);
    // Vector3f dims = props.getVector3("dimensions", Vector3f(0.4)).cwiseAbs();
    // Vector3f origin = props.getVector3("origin", Vector3f(0.f));
    // m_maxDensity = std::max(0.0f, props.getFloat("max_density", 1.f));
    // m_invDensityMax = m_maxDensity > Epsilon  ? 1.f / m_maxDensity : 0.f;

    // bounds = BoundingBox3f(origin - dims, origin + dims);
}

//Homogeneous transmission
// Color3f HomogeneousMedium::Tr(const Ray3f &ray, Sampler &sampler) const {
//     return exp(-m_sigma_t * std::min(ray.maxt * ray.Length(), MaxFloat))
// }
Color3f HomogeneousMedium::Tr(const Point3f &a, const Point3f &b) const {
    float norm = (a - b).norm();
    return {exp(-m_sigma_t.x() * norm), exp(-m_sigma_t.y() * norm), exp(-m_sigma_t.z() * norm)};
}

std::string Medium::toString() const {
    return "medium";
}

NORI_REGISTER_CLASS(Medium, "medium")


NORI_NAMESPACE_END
