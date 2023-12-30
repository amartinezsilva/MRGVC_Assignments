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

#include <nori/phase_function.h>
#include <nori/frame.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN


/**
 * \brief HenyeyGreenstein phase function
 */
class HenyeyGreenstein : public PhaseFunction {
public:
    HenyeyGreenstein(const PropertyList &propList) {
        m_g = propList.getFloat("g", 0.f);
    }


    /// Compute the density of \ref sample() wrt. solid angles
    float pdf(PhaseFunctionQueryRecord &pRec) const {

        float sqrtFrac = 1.0f + m_g * m_g + 2.0f * m_g * pRec.wi.dot(pRec.wo);
        float pdf =  INV_FOURPI * ((1.0f - m_g * m_g) / pow(sqrtFrac, 1.5f));

        return pdf;
    }

    /// Draw a a sample from the BRDF model
    float sample(PhaseFunctionQueryRecord &pRec, const Point2f &sample) const {
        
        float costheta;
        if(abs(m_g) <= Epsilon){
            costheta = 1.f - 2*sample.x();
        }

        else{
            float scale = 1.0f / (2.0f * m_g);  //negative?
            float fraction = (1.0f - m_g*m_g) / (1.0f - m_g + 2.0f * m_g * std::max(sample.x(), 0.001f));
            costheta =  scale * (1.0f +  m_g * m_g - fraction * fraction);
        }

        float phi = 2.0 * M_PI * sample.y();

        float sintheta = sqrt(std::max(0.f, 1 - costheta * costheta));

        Vector3f v_1, v_2;
        coordinateSystem(pRec.wi, v_1, v_2);
        pRec.wo = sintheta * cos(phi) * pRec.wi + sintheta * sin(phi) * v_2 + cos(phi) * v_1;
        
        return pdf(pRec);
    }


    /// Return a human-readable summary
    std::string toString() const {
        return tfm::format(
            "HenyeyGreenstein[\n"
            "  g = %f\n"
            "]", m_g);
    }


private:
    float m_g;
};

NORI_REGISTER_CLASS(HenyeyGreenstein, "henyeyGreenstein");
NORI_NAMESPACE_END