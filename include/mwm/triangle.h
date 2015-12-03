#ifndef _TRIANGLE_H_
#define _TRIANGLE_H_

#include <opencv2/core/core.hpp>

namespace mwm
{

struct Triangle
{

    Triangle(unsigned int i1_, unsigned int i2_, unsigned int i3_, const cv::Vec3b& clr_ = cv::Vec3b(255, 255, 255))
        : i1(i1_), i2(i2_), i3(i3_), color(clr_) {}
    unsigned int i1, i2, i3;
    cv::Vec3b color;

};

} // end namespace mwm

#endif
