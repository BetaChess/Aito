#include <iostream>

#include "logger.h"
#include "geometry.h"


int main()
{
	aito::Logger::init();

	aito::Vec3f a;

	aito::Transform::make_lookat({ 0, 0, 0 }, { 1, 1, 1 }, {1, 1, 1});
	

	return 0;
}