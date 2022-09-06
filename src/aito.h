#ifndef AITO_H
#define AITO_H

#include <cmath>

#include "logger.h"

namespace aito
{

// Float Type Definitions
#ifdef AITO_FLOAT_AS_DOUBLE
using Float = double;
#else
using Float = float;
#endif

#ifdef AITO_FLOAT_AS_DOUBLE
using FloatBits = uint64_t;
#else
using FloatBits = uint32_t;
#endif  // AITO_FLOAT_AS_DOUBLE
static_assert(sizeof(Float) == sizeof(FloatBits),
              "Float and FloatBits must have the same size");


#define MaxFloat std::numeric_limits<Float>::max()
#define Infinity std::numeric_limits<Float>::infinity()
#define MachineEpsilon (std::numeric_limits<Float>::epsilon() * 0.5)

static constexpr Float ShadowEpsilon = 0.0001f;
static constexpr Float Pi = 3.14159265358979323846;
static constexpr Float InvPi = 0.31830988618379067154;
static constexpr Float Inv2Pi = 0.15915494309189533577;
static constexpr Float Inv4Pi = 0.07957747154594766788;
static constexpr Float PiOver2 = 1.57079632679489661923;
static constexpr Float PiOver4 = 0.78539816339744830961;
static constexpr Float Sqrt2 = 1.41421356237309504880;

template<typename T>
[[nodiscard]] constexpr T lerp(Float t, T v1, T v2) { return (1 - t) * v1 + t * v2; };

inline Float gamma(int n)
{
    return (n * MachineEpsilon) / (1 - n * MachineEpsilon);
}


}





#endif 
