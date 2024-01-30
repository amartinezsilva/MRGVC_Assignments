/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    v1 - Dec 01 2020
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

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>
#include <nori/texture.h>
#include <nori/reflectance.h>


#include <cstring>
#include <string>

#include "MERLbrdfRead.h"

NORI_NAMESPACE_BEGIN


class MERLv1 : public BSDF {
public:
    MERLv1(const PropertyList &propList) {

        /* RMS surface roughness */
        m_alpha = new ConstantSpectrumTexture(propList.getFloat("alpha", 0.1f));
        
        path = "/home/luis/MRGVC_Assignments/MSAP/project/scenes/project/blender/xml/";
	    m_name = propList.getString("merl_coating", "bsdf_binaries/black-soft-plastic");
        path = path + m_name + ".binary";
        configure();
    }

    void configure() {
		
		// try to load the MERL BRDF data
		if(!read_brdf(path.c_str(), m_data))
		  std::cout << "Unable to find " << m_name.c_str() << std::endl;

        else std::cout << "bsdf read successfully" << std::endl;
		
        }

    /// Evaluate the BRDF model
    Color3f eval(const BSDFQueryRecord &bRec) const {
        /* This is a smooth BRDF -- return zero if the measure
           is wrong, or when queried for illumination on the backside */
        
        if (bRec.measure != ESolidAngle
            || Frame::cosTheta(bRec.wi) <= 0
            || Frame::cosTheta(bRec.wo) <= 0)
            return Color3f(0.0f);

        // // eval
        // double r,g,b;
        // double twi = acos(bRec.wi.z());
        // double two = acos(bRec.wo.z());
        // double pwi = atan2(bRec.wi.y(), bRec.wi.x());
        // double pwo = atan2(bRec.wo.y(), bRec.wo.x());

        // lookup_brdf_val(m_data, twi, pwi, two, pwo, r, g, b);
        // Color3f result(r,g,b);

        // return result;

        //roughConductor Eval

        Vector3f wh = (bRec.wi + bRec.wo).normalized();
        if (wh.x() == 0 && wh.y() == 0 && wh.z() == 0)
            return Color3f(0.0f);

        float alpha = m_alpha->eval(bRec.uv).getLuminance();
        
        // eval
        double r_h,g_h,b_h;
        double twi_h = acos(bRec.wi.z());
        double two_h = acos(wh.z());
        double pwi_h = atan2(bRec.wi.y(), bRec.wi.x());
        double pwo_h = atan2(wh.y(), wh.x());

        lookup_brdf_val(m_data, twi_h, pwi_h, two_h, pwo_h, r_h, g_h, b_h);
        
        float D = Reflectance::BeckmannNDF(wh, alpha);
        Color3f F = Reflectance::fresnel(Frame::cosTheta(bRec.wi), Color3f(r_h,g_h,b_h));
        float Gi = Reflectance::G1(bRec.wi, wh, alpha);
        float Go = Reflectance::G1(bRec.wo, wh, alpha);
        float G = Gi*Go;
        float denominator = 4 * Frame::cosTheta(bRec.wi) * Frame::cosTheta(bRec.wo);

        Color3f fr = D*F*G/denominator;

        return fr;

    }

    /// Compute the density of \ref sample() wrt. solid angles
    float pdf(const BSDFQueryRecord &bRec) const {
        /* This is a smooth BRDF -- return zero if the measure
            is wrong, or when queried for illumination on the backside */
        if (bRec.measure != ESolidAngle
            || Frame::cosTheta(bRec.wi) <= 0
            || Frame::cosTheta(bRec.wo) <= 0)
            return 0.0f;

        Vector3f wh = (bRec.wi + bRec.wo).normalized();
        return Warp::squareToBeckmannPdf(wh, m_alpha->eval(bRec.uv).getLuminance());
    }

    /// Draw a a sample from the BRDF model
    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const {
        
        if (Frame::cosTheta(bRec.wi) <= 0) {
            bRec.wo = Vector3f(1, 0, 0);
            return Color3f(0.0f);
        }

        bRec.measure = ESolidAngle;

        Vector3f wh  = Warp::squareToBeckmann(sample, m_alpha->eval(bRec.uv).getLuminance()).normalized();

        bRec.wo = 2.0f * (bRec.wi.dot(wh)) * wh - bRec.wi;
        bRec.eta = 1.0f;

        if(pdf(bRec) <= Epsilon)
            return Color3f(0.0f);

        return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);
    }

    bool isDiffuse() const {
        /* While microfacet BRDFs are not perfectly diffuse, they can be
           handled by sampling techniques for diffuse/non-specular materials,
           hence we return true here */
        return true;
    }

    /// Return a human-readable summary
   std::string toString() const {
        return tfm::format(
            "RoughConductor[\n"
            "  alpha = %f,\n"
            "  coating = %s \n"
            "]",
            m_alpha->toString(),
            m_name
        );
    }
    void addChild(NoriObject* obj, const std::string& name = "none") {
        switch (obj->getClassType()) {
        case ETexture:
            if (name == "alpha")
            {
                delete m_alpha;
                m_alpha = static_cast<Texture*>(obj);
            }
            else
                throw NoriException("RoughConductor::addChild(<%s>,%s) is not supported!",
                    classTypeName(obj->getClassType()), name);
            break;
        default:
            throw NoriException("RoughConductor::addChild(<%s>) is not supported!",
                classTypeName(obj->getClassType()));
        }
    }


    EClassType getClassType() const { return EBSDF; }
    
private:
    double * m_data;
    std::string m_name;
    std::string path;
    Texture * m_alpha;
};



class MERLv2 : public BSDF {
public:
    MERLv2(const PropertyList &propList) {
        
        path = "/home/luis/MRGVC_Assignments/MSAP/project/scenes/project/blender/xml/";
	    m_name = propList.getString("merl_coating", "bsdf_binaries/blue-acrylic");
        path = path + m_name + ".binary";
        configure();
    }

    void configure() {
		
		// try to load the MERL BRDF data
		if(!read_brdf(path.c_str(), m_data))
		  std::cout << "Unable to find " << m_name.c_str() << std::endl;

        else std::cout << "bsdf read successfully" << std::endl;
		
        }

    /// Evaluate the BRDF model
    Color3f eval(const BSDFQueryRecord &bRec) const {
        /* This is a smooth BRDF -- return zero if the measure
           is wrong, or when queried for illumination on the backside */
        
        if (bRec.measure != ESolidAngle
            || Frame::cosTheta(bRec.wi) <= 0
            || Frame::cosTheta(bRec.wo) <= 0)
            return Color3f(0.0f);

        // eval
        double r,g,b;
        double twi = acos(bRec.wi.z());
        double two = acos(bRec.wo.z());
        double pwi = atan2(bRec.wi.y(), bRec.wi.x());
        double pwo = atan2(bRec.wo.y(), bRec.wo.x());

        lookup_brdf_val(m_data, twi, pwi, two, pwo, r, g, b);
        Color3f result(r,g,b);

        return result * INV_PI;

    }

    /// Compute the density of \ref sample() wrt. solid angles
    float pdf(const BSDFQueryRecord &bRec) const {
        /* This is a smooth BRDF -- return zero if the measure
            is wrong, or when queried for illumination on the backside */
        if (bRec.measure != ESolidAngle
            || Frame::cosTheta(bRec.wi) <= 0
            || Frame::cosTheta(bRec.wo) <= 0)
            return 0.0f;

        return INV_PI * Frame::cosTheta(bRec.wo);
    }

    /// Draw a a sample from the BRDF model
    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const {
        
        if (Frame::cosTheta(bRec.wi) <= 0) {
            bRec.wo = Vector3f(1, 0, 0);
            return Color3f(0.0f);
        }

        bRec.measure = ESolidAngle;

        /* Warp a uniformly distributed sample on [0,1]^2
           to a direction on a cosine-weighted hemisphere */
        bRec.wo = Warp::squareToCosineHemisphere(sample);

        /* Relative index of refraction: no change */
        bRec.eta = 1.0f;

        if(pdf(bRec) <= Epsilon)
            return Color3f(0.0f);

        return eval(bRec);
    }

    bool isDiffuse() const {
        /* While microfacet BRDFs are not perfectly diffuse, they can be
           handled by sampling techniques for diffuse/non-specular materials,
           hence we return true here */
        return true;
    }

    /// Return a human-readable summary
   std::string toString() const {
        return tfm::format(
            "MERLv2[\n"
            "  coating = %s \n"
            "]",
            m_name
        );
    }
    void addChild(NoriObject* obj, const std::string& name = "none") {
        switch (obj->getClassType()) {
        default:
            throw NoriException("RoughConductor::addChild(<%s>) is not supported!",
                classTypeName(obj->getClassType()));
        }
    }


    EClassType getClassType() const { return EBSDF; }
    
private:
    double * m_data;
    std::string m_name;
    std::string path;
};


NORI_REGISTER_CLASS(MERLv1, "merl_v1");
NORI_REGISTER_CLASS(MERLv2, "merl_v2");
NORI_NAMESPACE_END
