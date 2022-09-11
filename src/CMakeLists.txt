
project (aito)

set(SOURCES
	"main.cpp"
    "vecmath.h" 
    "vecmath.cpp"
    "aito.h" 
    "geometry.h" 
    "geometry.cpp"
    "logger.h"
    "logger.cpp"
 "interaction.h" "medium.h" "interaction.cpp" "shape.h" "shape.cpp")

add_executable(${PROJECT_NAME} ${SOURCES})

set_target_properties(${PROJECT_NAME} 
    PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
)

target_include_directories(${PROJECT_NAME} 
	PUBLIC ../vendor/spdlog/include
    PUBLIC ../vendor/glm
)