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


NORI_NAMESPACE_BEGIN

class EnvironmentEmitter : public Emitter {
public:
	EnvironmentEmitter(const PropertyList& props) {
		m_type = EmitterType::EMITTER_ENVIRONMENT;
		m_environment = 0;

		std::string m_environment_name = props.getString("filename", "null");
		m_lightToWorld = props.getTransform("toWorld", Transform());
        m_worldToLight = m_lightToWorld.getInverseMatrix();

		filesystem::path filename =
			getFileResolver()->resolve(m_environment_name);

		std::ifstream is(filename.str());
		if (!is.fail())
		{
			cout << "Loading Environment Map: " << filename.str() << endl;

			m_environment = new Bitmap(filename.str());
			cout << "Loaded " << m_environment_name << " - SIZE [" << m_environment->rows() << ", " << m_environment->cols() << "]" << endl;
		}
		m_radiance = props.getColor("radiance", Color3f(1.));
	}
	~EnvironmentEmitter()
	{
		if (m_environment)
			delete m_environment;
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

		// This function call can be done by bsdf sampling routines.
		// Hence the ray was already traced for us - i.e a visibility test was already performed.
		// Hence just check if the associated normal in emitter query record and incoming direction are not backfacing
		if (!m_environment)
			return m_radiance;

		Vector3f w = (m_worldToLight * lRec.wi).normalized();

		float phi = atan2(w[2], w[0]);
		float theta = acos(w[1]);
		if (phi < 0) phi += 2 * M_PI;

		float x = phi / (2 * M_PI);
		float y = (theta) / M_PI;


		return m_environment->eval(Point2f(x, y))* m_radiance;
	}

	virtual Color3f sample(EmitterQueryRecord& lRec, const Point2f& sample, float optional_u) const {
				
		//lRec.wi = m_lightToWorld * Vector3f(sinTheta * cosPhi, sinTheta * sinPhi, cosTheta);
		lRec.wi = Warp::squareToUniformSphere(sample);
        lRec.n = -lRec.wi;
        lRec.pdf = pdf(lRec);
		return eval(lRec);
	}

	// Returns probability with respect to solid angle given by all the information inside the emitterqueryrecord.
	// Assumes all information about the intersection point is already provided inside.
	// WARNING: Use with care. Malformed EmitterQueryRecords can result in undefined behavior. Plus no visibility is considered.
	virtual float pdf(const EmitterQueryRecord& lRec) const {
		// Since you're sampling uniformly on the sphere, the pdf is constant
    	// You can use the pdf function from the Warp class for the squareToUniformSphere function
 		//Vector3f w = (m_worldToLight * lRec.wi).normalized();

		return Warp::squareToUniformSpherePdf(lRec.wi);

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
	Bitmap *m_environment;
	std::string m_environment_name;
};

NORI_REGISTER_CLASS(EnvironmentEmitter, "environment")
NORI_NAMESPACE_END
