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

#include "MERLbrdfRead.h"

NORI_NAMESPACE_BEGIN

/**
 * \brief Diffuse / Lambertian BRDF model
 */
class MERL : public BSDF {
public:
    MERL(const PropertyList &propList) {
        m_importance = NULL;
	    m_name = propList.getString("binary", "blue-acrylic.binary");
        configure();
    }

    void configure() {
		// check if importance sampling-guide as been specified
		if(m_importance == NULL)
		  std::cout << "You must specify a BSDF to guide importance sampling MERL BRDF \"%s\".", m_name.c_str() << std::endl;
		
		// try to load the MERL BRDF data
		if(!read_brdf(m_name.c_str(), m_data))
		  std::cout << "Unable to find \"%s\".", m_name.c_str() << std::endl;
		
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

        return result * Frame::cosTheta(bRec.wo);
    }

    /// Compute the density of \ref sample() wrt. solid angles
    float pdf(const BSDFQueryRecord &bRec) const {
        return m_importance->pdf(bRec);
    }

    /// Draw a a sample from the BRDF model
    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const {
        m_importance->sample(bRec, sample);
        if (pdf(bRec) == 0 || Frame::cosTheta(bRec.wo) <= 0)
            return Color3f(0.0f);
        else
            return eval(bRec) / pdf(bRec);
    }

    bool isDiffuse() const {
        return true;
    }

    /// Return a human-readable summary
    std::string toString() const {
        return tfm::format(
            "MERL[\n"
            "  m_importance = %s\n"
            "]", m_importance->toString());
    }

    void addChild(NoriObject* obj, const std::string& name = "none") {
        switch (obj->getClassType()) {
        case EBSDF:
            if (name == "m_importance")
            {
                delete m_importance;
                m_importance = static_cast<BSDF*>(obj);
            }
            else
                throw NoriException("MERL::addChild(<%s>,%s) is not supported!",
                classTypeName(obj->getClassType()), name);
            break;

        default:
            throw NoriException("MERL::addChild(<%s>) is not supported!",
                classTypeName(obj->getClassType()));
        }
    }


    EClassType getClassType() const { return EBSDF; }
    
private:
    BSDF *  m_importance;
    std::vector<double> m_data;
    std::string m_name;
};

NORI_REGISTER_CLASS(MERL, "merl");
NORI_NAMESPACE_END
