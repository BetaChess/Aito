#include "interaction.h"

namespace aito
{

Interaction::Interaction(const Point3f& p, const Normal3f& n, const Vec3f& p_err, const Vec3f& wo, Float time, const MediumInterface& medium_interface)
	: p(p), time(time), p_error(p_err), wo(wo), n(n), medium_interface(medium_interface)
{}

SurfaceInteraction::SurfaceInteraction(const Point3f& p,
									   const Vec3f& p_err, const Point2f& uv, const Vec3f& wo,
									   const Vec3f& dpdu, const Vec3f& dpdv,
									   const Normal3f& dndu, const Normal3f& dndv,
									   Float time, const Shape* shape)
	: Interaction(p, Normal3f(cross(dpdu, dpdv).normalized()), p_err, wo, time, nullptr),
	uv(uv), dpdu(dpdu), dpdv(dpdv), dndu(dndu), dndv(dndv), shape(shape),
	shading{n, dpdu, dpdv, dndu, dndv}
{
	if (shape && (shape->reverse_orientation ^ shape->transform_swaps_handedness))
	{
		n *= -1;
		shading.n *= -1;
	}
}

void SurfaceInteraction::set_shading_geometry(const Vec3f& dpdus, const Vec3f& dpdvs, 
											  const Normal3f& dndus, const Normal3f& dndvs, 
											  bool orientation_is_autoritative)
{
	// Compute shading n
	shading.n = Normal3f(cross(dpdus, dpdvs).normalized());
	if (shape && (shape->reverse_orientation ^ shape->transform_swaps_handedness))
	{
		shading.n *= -1;
	}
	if (orientation_is_autoritative)
		n = face_forward(n, static_cast<Vec3f>(shading.n));
	else
		shading.n = face_forward(shading.n, static_cast<Vec3f>(n));

	// Init shading partial derivative values
	shading.dpdu = dpdus;
	shading.dpdv = dpdvs;
	shading.dndu = dndus;
	shading.dndv = dndvs;
}

}
