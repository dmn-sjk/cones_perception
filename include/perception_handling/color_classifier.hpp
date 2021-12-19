#ifndef COLOR_CLASSIFIER_HPP
#define COLOR_CLASSIFIER_HPP

#include <map>

namespace perception_handling {

    typedef enum {
        kUnknownColor = 0,
        kYellow,
        kBlue,
        kOrange,
        kNumberOfColors
    } Color;

    std::map<Color, uint8_t> colors_to_intensities = {{kUnknownColor, 0}, 
                                                      {kYellow, 30}, 
                                                      {kBlue, 60}, 
                                                      {kOrange, 90}};

} // namespace perception_handling

#endif  // COLOR_CLASSIFIER_HPP