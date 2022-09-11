#include "aito.h"

#include "shape.h"


namespace aito
{



Shape::Shape(const Transform* object_to_world, const Transform* world_to_object, bool reverse_orientation)
	: object_to_world_T(object_to_world), world_to_object_T(world_to_object_T), 
	reverse_orientation(reverse_orientation), 
	transform_swaps_handedness(object_to_world_T->swaps_handedness())
{}

Bounds3f Shape::world_bound() const
{
	return (*object_to_world_T)(object_bound());
}

}
