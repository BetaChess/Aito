#ifndef SHAPE_H
#define SHAPE_H

#include "geometry.h"

namespace aito
{

class Shape
{
public:
	const Transform *object_to_world_T, *world_to_object_T;
	const bool reverse_orientation;
	const bool transform_swaps_handedness;

public:

	Shape(const Transform* object_to_world, const Transform* world_to_object, bool reverse_orientation);

	// Methods

	/// <returns>The bounds in object space. </returns>
	virtual Bounds3f object_bound() const = 0;
	/// <returns>The bounds in world space. </returns>
	virtual Bounds3f world_bound() const;

};

}

#endif // SHAPE_H