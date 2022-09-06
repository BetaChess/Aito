#include <iostream>

#include "logger.h"
#include "geometry.h"



int main()
{
	aito::Logger::init();

	aito::Vec3f a;

	aito::Transform::make_lookat({ 0, 0, 0 }, { 1, 1, 1 }, {1, 1, 1});

	
	

	
	//m[0][3] = 5;
	//m[0] = glm::vec4(1, 2, 3, 4);
	//auto f = m[0] + m[2];

	return 0;
}