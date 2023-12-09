#pragma once

#include <nori/object.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN


class PhaseFunction : public NoriObject {

    public:
        virtual float p(const Vector3f &wo, const Vector3f &wi) const = 0;
        virtual float sample_p(const Vector3f &wo, Vector3f *wi,
                                const Point2f &u) const = 0;

        EClassType getClassType() const { return EPhaseFunction; }
        std::string toString() const override;

};



class HenyeyGreenstein : public PhaseFunction {

    public:

        HenyeyGreenstein(float g) : m_g(g) { }
        float p(const Vector3f &wo, const Vector3f &wi) const;
        float sample_p(const Vector3f &wo, Vector3f *wi, const Point2f &sample) const;

        EClassType getClassType() const { return EPhaseFunction; }
        std::string toString() const override;


    private:

        const float m_g;


};




NORI_NAMESPACE_END