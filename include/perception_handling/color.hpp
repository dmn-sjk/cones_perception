#ifndef COLOR_HPP
#define COLOR_HPP

#include "stdint.h"

namespace perception_handling {

    typedef enum {
        kUnknownColor = 0,
        kYellow,
        kBlue,
        kOrange,
        // kBiggerOrange,
        kNumberOfColors
    } Color;

} // namespace perception_handling

#endif  // COLOR_HPP