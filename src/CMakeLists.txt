
project (aito)

set(SOURCES
	"main.cpp"
    "vecmath.h" 
    "vecmath.cpp"
    "aito.h" 
    "geometry.h" "geometry.cpp")

add_executable(${PROJECT_NAME} ${SOURCES})

set_target_properties(${PROJECT_NAME} 
    PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
)
