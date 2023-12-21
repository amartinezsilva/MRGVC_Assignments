#include <nori/medium.h>

NORI_NAMESPACE_BEGIN


class HomogeneousMedium : public Medium {
public:
    HomogeneousMedium   (const PropertyList &propList) {
        m_sigmaA = propList.getColor("sigmaA", Color3f(0.5f));
        m_sigmaS = propList.getColor("sigmaS", Color3f(0.5f));
        m_sigmaT = m_sigmaA + m_sigmaS;
        m_albedo = m_sigmaS / m_sigmaT;
    }

    bool sampleDistance(const Ray3f &ray, MediumSamplingRecord &mRec, const Point2f &sample) const {
        float rand = sample(0);
        float sampledDistance;
        float samplingDensity;

        // extintion coefficient
        int channel = std::min((int) (sample(1) * 3.0f), 2);
        samplingDensity = m_sigmaT[channel];

        sampledDistance = -std::log(1.0f - rand) / samplingDensity;

        float distSurf = ray.maxt - ray.mint;
        bool success = true;
        if (sampledDistance < distSurf) {
            mRec.t = sampledDistance + ray.mint;
            mRec.p = ray(mRec.t);
            mRec.sigmaA = m_sigmaA;
            mRec.sigmaS = m_sigmaS;
            mRec.medium = this;

            /* Fail if there is no forward progress
               (e.g. due to roundoff errors) */
            if (mRec.p == ray.o)
                success = false;
        } else {
            sampledDistance = distSurf;
            success = false;
        }


        mRec.pdfFailure = 0.0f;
        mRec.pdfSuccess = 0.0f;
        for (int i=0; i<3; ++i) {
            float tmp = std::exp(-m_sigmaT[i] * sampledDistance);
            mRec.pdfFailure += tmp;
            mRec.pdfSuccess += m_sigmaT[i] * tmp;
        }
        mRec.pdfFailure /= 3.0f;
        mRec.pdfSuccess /= 3.0f;

        Color3f temp = (m_sigmaT * (-sampledDistance));
        mRec.transmittance = Color3f(expf(temp.r()), expf(temp.g()), expf(temp.b()));

        mRec.medium = this;
        if (mRec.transmittance.maxCoeff() < 1e-20)
            mRec.transmittance = Color3f(0.0f);

        return success;
    }
    
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
    void eval(const Ray3f &ray, MediumSamplingRecord &mRec) const {
        float distance = ray.maxt - ray.mint;

        mRec.pdfSuccess = 0;
        mRec.pdfFailure = 0;
        for (int i=0; i<3; ++i) {
            float temp = std::exp(-m_sigmaT[i] * distance);
            mRec.pdfSuccess += m_sigmaT[i] * temp;
            mRec.pdfFailure += temp;
        }
        mRec.pdfSuccess /= 3;
        mRec.pdfFailure /= 3;
        Color3f temp = m_sigmaT * (-distance);
        std::cout << "mRec.temp: " << expf(temp.g());
        mRec.transmittance = Color3f(expf(temp.r()), expf(temp.g()), expf(temp.b()));
        mRec.sigmaA = m_sigmaA;
        mRec.sigmaS = m_sigmaS;

        mRec.medium = this;
        // if (mRec.transmittance.maxCoeff() < 1e-20)
        //     std::cout << "pRec.pdf: " << mRec.transmittance;
        //     mRec.transmittance = Color3f(0.0f);

    }

    /**
     * \brief Compute the transmittance along a ray segment
     *
     * Computes the transmittance along a ray segment
     * [mint, maxt] associated with the ray. It is assumed
     * that the ray has a normalized direction value.
     */
    Color3f evalTransmittance(const Ray3f &ray, const Point2f &sample) const {
        float negLength = ray.mint - ray.maxt;
        Color3f transmittance;
        
        for (int i=0; i<3; ++i){
            if(m_sigmaT[i] != 0 && negLength != 0) {
                transmittance[i] = std::exp(m_sigmaT[i] * negLength);
            } else {
                cout << "transmittance: ";
                //cout << "transmittance: " << medium[0]->evalTransmittance(ray, sampler->next1D());
                transmittance[i] = 1.0f;
            }
        }
        return transmittance;
    }

    /// Determine whether the medium is homogeneous
    bool isHomogeneous() const  {
        return true;
    }


    /// Add a child
    void addChild(NoriObject *obj, const std::string& name) {
        switch (obj->getClassType()) {
            case EPhaseFunction:
                if (m_phaseFunction)
                    throw NoriException(
                        "Medium: tried to register multiple Phase functions!");
                m_phaseFunction = static_cast<PhaseFunction *>(obj);
                break;

            default:
                throw NoriException("Medium::addChild(<%s>) is not supported!",
                                    classTypeName(obj->getClassType()));
        }
    }

    /// Return a string representation
    std::string toString() const {
        return tfm::format(
                    "HomogeneousMedium[\n"
                    "  sigmaA = %s,\n"
                    "  sigmaS = %s,\n"
                    "  sigmaT = %s,\n"
                    "  albedo = %s,\n"
                    "]",
                    m_sigmaA,
                    m_sigmaS,
                    m_sigmaT,
                    m_albedo);
    }
};

NORI_REGISTER_CLASS(HomogeneousMedium, "homogeneous");
NORI_NAMESPACE_END
