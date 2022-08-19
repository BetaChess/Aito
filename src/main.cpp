#include <iostream>

#include "logger.h"
#include "geometry.h"



int main()
{
	aito::Logger::init();

	aito::Vec3f a;

	aito::Transform::make_lookat({ 0, 0, 0 }, { 1, 1, 1 }, {1, 1, 1});

	glm::mat4 t {
		1,  5,   9, 13, 
		2,  6,  10, 14, 
		3,  7,  11, 15, 
		4,  8,  12, 16};

	auto v = glm::column(t, 3);
	
	const aito::Mat4 m{
		1,  5,   9, 13,
		2,  6,  10, 14,
		3,  7,  11, 15,
		4,  8,  12, 16 };

	
	//m[0][3] = 5;
	//m[0] = glm::vec4(1, 2, 3, 4);
	auto f = m[0] + m[2];

	return 0;
}