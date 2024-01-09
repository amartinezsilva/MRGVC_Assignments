/*
	This file is part of Nori, a simple educational ray tracer
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

#include <nori/emitter.h>
#include <nori/bitmap.h>
#include <nori/warp.h>
#include <filesystem/resolver.h>
#include <fstream>
#include <nori/transform.h>
#include <nori/medium.h>


NORI_NAMESPACE_BEGIN

class EnvironmentMediumEmitter : public Emitter {
public:
	EnvironmentMediumEmitter(const PropertyList& props) {
		m_type = EmitterType::EMITTER_ENVIRONMENT;

		std::string m_environment_name = props.getString("filename", "null");
		m_lightToWorld = props.getTransform("toWorld", Transform());
        m_worldToLight = m_lightToWorld.getInverseMatrix();

		filesystem::path filename =
			getFileResolver()->resolve(m_environment_name);

		std::ifstream is(filename.str());
		if (!is.fail())
		{
			cout << "Loading Environment Map: " << filename.str() << endl;

			m_environment = Bitmap(filename.str());
			cout << "Loaded " << m_environment_name << " - SIZE [" << m_environment.rows() << ", " << m_environment.cols() << "]" << endl;
		}
		m_radiance = props.getColor("radiance", Color3f(1.));
		m_size = Vector2i(m_environment.rows(), m_environment.cols());    

		// Precompute the pdf
        m_pPhi = Eigen::MatrixXf(m_size.x(), m_size.y());
        for (int x = 0; x < m_size.x(); x++)
        {
            float sinTheta = sin(M_PI * float(x + 0.5f) / float(m_size.x()));
            for (int y = 0; y < m_size.y(); y++)
            {
                m_pPhi(x, y) = m_environment(x, y).getLuminance() * sinTheta;
            }
        }
        m_pTheta = Eigen::VectorXf(m_pPhi.rows());
        m_pTheta = m_pPhi.rowwise().sum();
        m_pTheta = m_pTheta / m_pTheta.sum();
        for (int i = 0; i < m_pPhi.rows(); i++)
        {
            if (m_pTheta(i) > 1e-5)
                m_pPhi.row(i) /= m_pTheta(i);
            else
                m_pPhi.row(i) *= 0;
        }

	}

	virtual std::string toString() const {
		return tfm::format(
			"AreaLight[\n"
			"  radiance = %s,\n"
			"  environment = %s,\n"
			"  lightToWorld = %s\n"
			"]",
			m_radiance.toString(),
			m_environment_name,
			indent(m_lightToWorld.toString(), 18));
	}

	// We don't assume anything about the visibility of points specified in 'ref' and 'p' in the EmitterQueryRecord.
	virtual Color3f eval(const EmitterQueryRecord& lRec) const {

		// Nearest neighbor interpolation
        Vector3f w = (m_worldToLight * lRec.wi).normalized();
        float theta = std::acos(clamp(w.z(), -1.f, 1.f));
        float p = std::atan2(w.y(), w.x());
        float phi = (p < 0) ? (p + 2 * M_PI) : p; 
        int x = round(theta * INV_PI * (m_size.x() - 1));
        int y = round(phi * INV_TWOPI * (m_size.y() - 1));
        x = clamp(x, 0, m_size.x()-1);
        y = clamp(y, 0, m_size.y()-1);
        return m_environment(x, m_size.y() -1 - y);
	}

	virtual Color3f sample(EmitterQueryRecord& lRec, const Point2f& sample, float optional_u) const {
				
		float theta = M_PI * sample.x(), phi = 2 * M_PI * sample.y();
        float cosTheta = std::cos(theta), sinTheta = std::sin(theta);
        float sinPhi = std::sin(phi), cosPhi = std::cos(phi);
        lRec.wi = m_lightToWorld * Vector3f(sinTheta * cosPhi, sinTheta * sinPhi, cosTheta);
        lRec.shadowRay = Ray3f(lRec.ref, lRec.wi, Epsilon, INFINITY, this->getMedium());
        if (pdf(lRec) > 1e-9)
            return eval(lRec)/pdf(lRec);
        else 
            return Color3f(0.f);
	}

	// Returns probability with respect to solid angle given by all the information inside the emitterqueryrecord.
	// Assumes all information about the intersection point is already provided inside.
	// WARNING: Use with care. Malformed EmitterQueryRecords can result in undefined behavior. Plus no visibility is considered.
	virtual float pdf(const EmitterQueryRecord& lRec) const {

		Vector3f w = (m_worldToLight * lRec.wi).normalized();
        float theta = std::acos(clamp(w.z(), -1.f, 1.f));
        float p = std::atan2(w.y(), w.x());
        float phi = (p < 0) ? (p + 2 * M_PI) : p; 
        float sinTheta = std::sin(theta);
        if (sinTheta == 0) return 0;
        
        int x = round(theta * INV_PI * (m_size.x() - 1));
        int y = round(phi * INV_TWOPI * (m_size.y() -1));
        x = clamp(x, 0, m_size.x()-1);
        y = clamp(y, 0, m_size.y()-1);
        float pdf = m_pTheta(x) * m_pPhi(x, m_size.y() -1 - y);
        
        return pdf / (2 * sinTheta) * INV_PI * INV_PI;
	}

	virtual bool isInfinity() const {return true;}


	// Get the parent mesh
	void setParent(NoriObject* parent)
	{
	}


protected:
	Color3f m_radiance;
    Transform m_worldToLight;
    Transform m_lightToWorld;
	Bitmap m_environment;
	std::string m_environment_name;
	Vector2i m_size;
	Eigen::VectorXf m_pTheta;
    Eigen::MatrixXf m_pPhi;

};

NORI_REGISTER_CLASS(EnvironmentMediumEmitter, "environment_medium")
NORI_NAMESPACE_END