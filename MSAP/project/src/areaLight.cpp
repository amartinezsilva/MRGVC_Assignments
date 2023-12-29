// /*
//     This file is part of Nori, a simple educational ray tracer

//     Copyright (c) 2015 by Wenzel Jakob
	
// 	v1 - Dec 01 2020
// 	Copyright (c) 2020 by Adrian Jarabo

//     Nori is free software; you can redistribute it and/or modify
//     it under the terms of the GNU General Public License Version 3
//     as published by the Free Software Foundation.

//     Nori is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//     GNU General Public License for more details.

//     You should have received a copy of the GNU General Public License
//     along with this program. If not, see <http://www.gnu.org/licenses/>.
// */

// #include <nori/emitter.h>
// #include <nori/warp.h>
// #include <nori/mesh.h>
// #include <nori/texture.h>

// NORI_NAMESPACE_BEGIN

// class AreaLight : public Emitter {
// public:
// 	AreaLight(const PropertyList &props) {
// 		m_type = EmitterType::EMITTER_AREA;
//         m_radiance = props.getColor("radiance", Color3f(10.0f, 10.0f, 10.0f));
// 		m_scale = props.getFloat("scale", 1.);
// 	}

// 	virtual std::string toString() const {
// 		return tfm::format(
// 			"AreaLight[\n"
// 			"  radiance = %s,\n"
// 			"  scale = %f,\n"
// 			"]",
// 			m_radiance->toString(), m_scale);
// 	}

// 	// We don't assume anything about the visibility of points specified in 'ref' and 'p' in the EmitterQueryRecord.
// 	virtual Color3f eval(const EmitterQueryRecord & lRec) const {
// 		if (!m_mesh)
// 			throw NoriException("There is no shape attached to this Area light!");

// 		// This function call can be done by bsdf sampling routines.
// 		// Hence the ray was already traced for us - i.e a visibility test was already performed.
// 		// Hence just check if the associated normal in emitter query record and incoming direction are not backfacing
		
        
//         // if (lRec.n.dot(-lRec.wi) > 0) {		// (-wi, emitter_normal) < 90º
// 		// 	return m_radiance->eval(lRec.uv); 
// 		// } else {
// 			return Color3f(0);
// 		// }
// 	}

// 	virtual Color3f sample(EmitterQueryRecord & lRec, const Point2f & sample, float optional_u) const {
// 		if (!m_mesh)
// 			throw NoriException("There is no shape attached to this Area light!");

// 		m_mesh->samplePosition(sample, lRec.p, lRec.n, lRec.uv);

// 		lRec = EmitterQueryRecord(this, lRec.ref, lRec.p, lRec.n, lRec.uv);
// 		lRec.pdf = pdf(lRec);
// 		if (lRec.pdf < Epsilon) {
// 			return Color3f(0.0f);
// 		} else {
// 			return eval(lRec) / lRec.pdf;
// 		}
// 	}

// 	// Returns probability with respect to solid angle given by all the information inside the emitterqueryrecord.
// 	// Assumes all information about the intersection point is already provided inside.
// 	// WARNING: Use with care. Malformed EmitterQueryRecords can result in undefined behavior. 
// 	//			Plus no visibility is considered.
// 	virtual float pdf(const EmitterQueryRecord &lRec) const {
// 		if (!m_mesh)
// 			throw NoriException("There is no shape attached to this Area light!");
		
// 		float pdf = m_mesh->pdf(lRec.p);
// 		return pdf * (lRec.dist * lRec.dist) / std::abs(lRec.n.dot(lRec.wi));
// 	}


// 	// Get the parent mesh
// 	void setParent(NoriObject *parent)
// 	{
// 		auto type = parent->getClassType();
// 		if (type == EMesh)
// 			m_mesh = static_cast<Mesh*>(parent);
// 	}

// 	// Set children
// 	// void addChild(NoriObject* obj, const std::string& name = "none") {
// 	// 	switch (obj->getClassType()) {
// 	// 	case ETexture:
// 	// 		if (name == "radiance")
// 	// 		{
// 	// 			delete m_radiance;
// 	// 			m_radiance = static_cast<Texture*>(obj);
// 	// 		}
// 	// 		else
// 	// 			throw NoriException("AreaLight::addChild(<%s>,%s) is not supported!",
// 	// 				classTypeName(obj->getClassType()), name);
// 	// 		break;

// 	// 	default:
// 	// 		throw NoriException("AreaLight::addChild(<%s>) is not supported!",
// 	// 			classTypeName(obj->getClassType()));
// 	// 	}
// 	// }

// private:
//     Color3f Power(const Scene *) const {return  Color3f(0.0f); }

//     bool isDeltaLight() const { return false; }


//     Color3f sampleL(Point3f &p,
//                     float pEpsilon,
//                     const Point2f &ls, /*float time,*/
//                     Vector3f *wi,
//                     float *pdf,
//                     VisibilityTester *vis) const {

//         Normal3f ns;
//         Point3f ps;
//         //sample the mesh
//         m_mesh->samplePositionSimple(ls,ps,ns);

//         //set the direction to the light
//         *wi = (ps - p).normalized();

//         //set the pdf of the sample
//         *pdf = m_mesh->Pdf(p, ps, ns, *wi);

//         if(isnan(*pdf)){
//             cout << "p = " << p << endl;
//             cout << "ps = " << ps << endl;
//             cout << "ns = " << ns << endl;
//             cout << "wi = " << *wi << endl;
//         }


//         //init the visibility tester
//         vis->SetSegment(p, pEpsilon, ps , Epsilon);

//         //check if a backfac is hitn
//         if(ns.dot(-*wi) < 0.0) return Color3f(0.0, 0.0, 0.0);

//         return 0.0 < *pdf? Color3f(15.0f, 15.0f, 15.0f) : Color3f(0.0, 0.0, 0.0);

//     }

//     //TEMPORY
//     Color3f sampleL(const Vector3f &d, const Normal3f &n, const Intersection &its) const {
//         return  (d.dot(n) > 0) ? Color3f(15.0f, 15.0f, 15.0f) : Color3f(0.0f, 0.0f, 0.0f);
//     }

//     //TEMPORY
//     Color3f sampleL(const Vector3f &d) const {
//             //std::cout << "area hit" << std::endl;
//             return Color3f(15.0f, 15.0f, 15.0f);
//     }

//     void setMesh(Mesh* mesh){
//         m_mesh = mesh;
//     }

//     Color3f radiance() const {
//         return Color3f(15.0f, 15.0f, 15.0f);
//     }
//     float pdf(Point3f p, Vector3f w, Point3f hitPoint, Normal3f n) const {
//         return m_mesh->Pdf(p, hitPoint, n, w);
//     }

// protected:
// 	Color3f* Color3f(15.0f, 15.0f, 15.0f);
// 	float m_scale;
// }



// NORI_REGISTER_CLASS(AreaLight , "areaLight");
// NORI_NAMESPACE_END