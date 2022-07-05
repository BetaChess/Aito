#ifndef AITO_H
#define AITO_H

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
}


#endif 
