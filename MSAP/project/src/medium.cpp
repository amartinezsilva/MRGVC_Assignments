#include <nori/medium.h>

NORI_NAMESPACE_BEGIN


class HomogeneousMedium : public Medium {
public:
    HomogeneousMedium   (const PropertyList &propList) {
        m_sigmaA = propList.getColor("sigmaA", Color3f(0.5f));
        m_sigmaS = propList.getColor("sigmaS", Color3f(0.5f));
        m_sigmaT = m_sigmaA + m_sigmaS;
    }

    Color3f sample(const Ray3f &ray, const Point2f &sample, MediumInteraction &mRec) const {
        
        // Sample one of the three channels and distance along the ray
        int channel = std::min(int(sample.x() * 3), 3 - 1);
        float dist = -std::log(1 - sample.y()) / m_sigmaT(channel);
        float t = std::min(dist * ray.d.norm(), ray.maxt);
        bool sampleMediumSuccess = t < ray.maxt;
        if (sampleMediumSuccess)
            mRec = MediumInteraction(ray(t), t, -(ray.d).normalized(), this->getPhaseFunction(), this);

        // Compute the trasmittance and sampling density
        Color3f Tr = exp(- m_sigmaT * std::min(t, INFINITY) * ray.d.norm()); //INFINITY OR MAXFLOAT
            
        // Return weighting factor for scattering from homogeneous medium
        Color3f density = sampleMediumSuccess ? (m_sigmaT * Tr) : Tr;
        float pdf = 0;
        for (int i = 0; i < 3; i++)
            pdf += density(i);
        pdf *= 1.f / 3;
        
        if (sampleMediumSuccess) Tr *= m_sigmaS;

        return Tr / pdf;        
    }

    /**
     * \brief Compute the transmittance along a ray
     *
     */
    Color3f Tr(const Ray3f &ray) const {
        float d = std::min(ray.maxt, INFINITY); // MAXFLOAT or INFINITY
        return exp(- m_sigmaT * d);
    }

    /// Return a string representation
    std::string toString() const {
        return tfm::format(
                    "HomogeneousMedium[\n"
                    "  sigmaA = %s,\n"
                    "  sigmaS = %s,\n"
                    "  sigmaT = %s,\n"
                    "]",
                    m_sigmaA.toString(),
                    m_sigmaS.toString(),
                    m_sigmaT.toString());
    }

private:
    Color3f m_sigmaA, m_sigmaS, m_sigmaT;
};

NORI_REGISTER_CLASS(HomogeneousMedium, "homogeneous");
NORI_NAMESPACE_END