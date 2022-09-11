#ifndef INTERACTION_H
#define INTERACTION_H

#include "aito.h"
#include "medium.h"
#include "vecmath.h"

namespace aito
{

struct Interaction
{
public:
	Point3f p;
	Float time;
	Vec3f p_error;
	Vec3f wo;
	Normal3f n;
	MediumInterface medium_interface;

public:

	inline Interaction() : time(0) {};
	Interaction(const Point3f& p, const Normal3f& n, const Vec3f& p_err,
				const Vec3f& wo, Float time,
				const MediumInterface& medium_interface);

	// methods

	inline bool is_surface_interaction() const
	{
		return n != Normal3f();
	}

};

class Shape;
class Primitive;
class BSDF;
class BSSRDF;

class SurfaceInteraction : public Interaction
{
public:
	Point2f uv;
	Vec3f dpdu, dpdv;
	Normal3f dndu, dndv;
	const Shape* shape = nullptr;

	struct
	{
		Normal3f n;
		Vec3f dpdu, dpdv;
		Normal3f dndu, dndv;
	} shading;

	const Primitive* primitive = nullptr;
	BSDF* bsdf = nullptr;
	BSSRDF* bssrdf = nullptr;
	mutable Vec3f dpdx, dpdy;
	mutable Float dudx = 0, dvdx = 0, dudy = 0, dvdy = 0;

public:

	inline SurfaceInteraction() {};
	SurfaceInteraction(const Point3f& p, const Vec3f& p_err, 
					   const Point2f& uv, const Vec3f& wo,
					   const Vec3f& dpdu, const Vec3f& dpdv,
					   const Normal3f& dndu, const Normal3f& dndv,
					   Float time, const Shape* shape);

	// Methods

	void set_shading_geometry(const Vec3f& dpdus, const Vec3f& dpdvs, 
							  const Normal3f& dndus, const Normal3f& dndvs,
							  bool orientation_is_autoritative);

};


}

#endif // INTERACTION_H
